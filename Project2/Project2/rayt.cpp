#include "rayt.h"
#include <time.h>

#define NUM_THREAD 4

namespace rayt {

	class Shape;
	typedef std::shared_ptr<Shape> ShapePtr;
	class Material;
	typedef std::shared_ptr<Material> MaterialPtr;

	//-------------------------------------------------------------------------

	class HitRec {

	public:

		float t;
		float u;
		float v;
		vec3 p;
		vec3 n;
		MaterialPtr mat;

	};

	//-------------------------------------------------------------------------

	class ScatterRec
	{
	public:
		Ray ray;
		vec3 albedo;
	};

	class Material
	{
	public:
		virtual bool scatter(const Ray& r, const HitRec& hrec, ScatterRec& srec) const = 0;
		virtual vec3 emitted(const Ray& r, const HitRec& hrec) const { return vec3(0); }
	};

	class Lambertian : public Material
	{
	public:
		Lambertian(const TexturePtr& a)
			:m_albedo(a){
		}

		virtual bool scatter(const Ray& r, const HitRec& hrec, ScatterRec& srec) const override {
			vec3 target = hrec.p + hrec.n + random_in_unit_sphere();
			srec.ray = Ray(hrec.p, target - hrec.p);
			srec.albedo = m_albedo->value(hrec.u, hrec.v, hrec.p);
			return true;
		}

	private:
		TexturePtr m_albedo;
	};

	class Metal : public Material
	{
	public:
		Metal(const TexturePtr& a, float fuzz) 
			:m_albedo(a)
			,m_fuzz(fuzz){
		}
		virtual bool scatter(const Ray& r, const HitRec& hrec, ScatterRec& srec) const override {
			vec3 reflected = reflect(normalize(r.direction()), hrec.n);
			reflected += m_fuzz * random_in_unit_sphere();
			srec.ray = Ray(hrec.p, reflected);
			srec.albedo = m_albedo->value(hrec.u, hrec.v, hrec.p);
			return dot(srec.ray.direction(), hrec.n) > 0;
		}

	private:
		TexturePtr m_albedo;
		float m_fuzz;
	};

	class Dielectric : public Material
	{
	public:
		Dielectric(float ri)
			:m_ri(ri) {
		}
		virtual bool scatter(const Ray& r, const HitRec& hrec, ScatterRec& srec) const override {
			vec3 outward_normal;
			vec3 reflected = reflect(r.direction(), hrec.n);
			float ni_over_nt;
			float reflect_prob;
			float cosine;
			if (dot(r.direction(), hrec.n) > 0) {
				outward_normal = -hrec.n;
				ni_over_nt = m_ri;
				cosine = m_ri * dot(r.direction(), hrec.n) / length(r.direction());
			}
			else {
				outward_normal = hrec.n;
				ni_over_nt = recip(m_ri);
				cosine = -dot(r.direction(), hrec.n) / length(r.direction());
			}

			srec.albedo = vec3(1);

			vec3 refracted;
			if (refract(-r.direction(), outward_normal, ni_over_nt, refracted)) {
				reflect_prob = schlick(cosine, m_ri);
			}
			else {
				reflect_prob = 1;
			}

			if (drand48() < reflect_prob) {
				srec.ray = Ray(hrec.p, reflected);
			}
			else {
				srec.ray = Ray(hrec.p, refracted);
			}

			return true;
		}

	private:
		float m_ri;
	};

	class DiffuseLight : public Material
	{
	public:
		DiffuseLight(const TexturePtr& emit)
			: m_emit(emit) {
		}

		virtual bool scatter(const Ray& r, const HitRec& grec, ScatterRec& srec) const override {
			return false;
		}

		virtual vec3 emitted(const Ray& r, const HitRec& hrec) const override {
			return m_emit->value(hrec.u, hrec.v, hrec.p);
		}

	private:
		TexturePtr m_emit;
	};

	//-------------------------------------------------------------------------

	class Shape {

	public:

		virtual bool hit(const Ray& r, float t0, float t1, HitRec& hrec) const = 0;

	};

	class ShapeList : public Shape {

	public:
		ShapeList() {}

		void add(const ShapePtr& shape) {

			m_list.push_back(shape);

		}

		virtual bool hit(const Ray& r, float t0, float t1, HitRec& hrec) const override {
			HitRec temp_rec;
			bool hit_anything = false;
			float closest_so_far = t1;
			for (auto& p : m_list) {
				if (p->hit(r, t0, closest_so_far, temp_rec)) {
					hit_anything = true;
					closest_so_far = temp_rec.t;
					hrec = temp_rec;
				}
			}
			return hit_anything;
		}

	private:
		std::vector<ShapePtr> m_list;

	};

	class Sphere : public Shape {

	public:
		Sphere() {}
		Sphere(const vec3& c, float r, const MaterialPtr& mat)
			: m_center(c)
			, m_radius(r)
			, m_material(mat){
		}

		virtual bool hit(const Ray& r, float t0, float t1, HitRec& hrec) const override {

			vec3 oc = r.origin() - m_center;
			float a = dot(r.direction(), r.direction());
			float b = 2.0f*dot(oc, r.direction());
			float c = dot(oc, oc) - pow2(m_radius);
			float D = b * b - 4 * a*c;

			if (D > 0) {
				float root = sqrtf(D);
				float temp = (-b - root) / (2.0f*a);
				if (temp < t1 && temp > t0) {
					hrec.t = temp;
					hrec.p = r.at(hrec.t);
					hrec.n = (hrec.p - m_center) / m_radius;
					hrec.mat = m_material;
					get_sphere_uv(hrec.p, hrec.u, hrec.v);
					return true;
				}

				temp = (-b + root) / (2.0f*a);
				if (temp < t1 && temp > t0) {
					hrec.t = temp;
					hrec.p = r.at(hrec.t);
					hrec.n = (hrec.p - m_center) / m_radius;
					hrec.mat = m_material;
					get_sphere_uv(hrec.p, hrec.u, hrec.v);
					return true;
				}
			}

			return false;
		}



	private:

		vec3 m_center;
		float m_radius;
		MaterialPtr m_material;
	};

	class Rect : public Shape
	{
	public:
		enum AxisType {
			kXY = 0,
			kXZ,
			kYZ
		};
		Rect() {}
		Rect(float x0, float x1, float y0, float y1, float k, AxisType axis, const MaterialPtr& m)
			: m_x0(x0)
			, m_x1(x1)
			, m_y0(y0)
			, m_y1(y1)
			, m_k(k)
			, m_axis(axis)
			, m_material(m) {
		}

		virtual bool hit(const Ray& r, float t0, float t1, HitRec& hrec) const override {
			int xi, yi, zi;
			vec3 axis;
			switch (m_axis) {
			case kXY: xi = 0; yi = 1; zi = 2; axis = vec3::zAxis(); break;
			case kXZ: xi = 0; yi = 2; zi = 1; axis = vec3::yAxis(); break;
			case kYZ: xi = 1; yi = 2; zi = 0; axis = vec3::xAxis(); break;
			}

			float t = (m_k - r.origin()[zi]) / r.direction()[zi];
			if (t < t0 || t > t1) {
				return false;
			}

			float x = r.origin()[xi] + t * r.direction()[xi];
			float y = r.origin()[yi] + t * r.direction()[yi];
			if (x < m_x0 || x > m_x1 || y < m_y0 || y > m_y1) {
				return false;
			}

			hrec.u = (x - m_x0) / (m_x1 - m_x0);
			hrec.v = (y - m_y0) / (m_y1 - m_y0);
			hrec.t = t;
			hrec.mat = m_material;
			hrec.p = r.at(t);
			hrec.n = axis;
			return true;
		}

	private:
		float m_x0, m_x1, m_y0, m_y1, m_k;
		AxisType m_axis;
		MaterialPtr m_material;
	};

	class FlipNormals : public Shape
	{
	public:
		FlipNormals(const ShapePtr& shape)
			: m_shape(shape) {
		}

		virtual bool hit(const Ray& r, float t0, float t1, HitRec& hrec) const override {
			if (m_shape->hit(r, t0, t1, hrec)) {
				hrec.n = -hrec.n;
				return true;
			}
			else {
				return false;
			}
		}
	private:
		ShapePtr m_shape;
		
	};

	class Box : public Shape
	{
	public:
		Box() {}
		Box(const vec3& p0, const vec3& p1, const MaterialPtr& m)
			: m_p0(p0)
			, m_p1(p1)
			, m_list(std::make_unique<ShapeList>()) {

			ShapeList* l = new ShapeList();
			l->add(std::make_shared<Rect>(p0.getX(), p1.getX(), p0.getY(), p1.getY(), p1.getZ(), Rect::kXY, m));
			l->add(std::make_shared<Rect>(p0.getX(), p1.getX(), p0.getY(), p1.getY(), p0.getZ(), Rect::kXY, m));
			l->add(std::make_shared<Rect>(p0.getX(), p1.getX(), p0.getZ(), p1.getZ(), p1.getY(), Rect::kXZ, m));
			l->add(std::make_shared<Rect>(p0.getX(), p1.getX(), p0.getZ(), p1.getZ(), p0.getY(), Rect::kXZ, m));
			l->add(std::make_shared<Rect>(p0.getY(), p1.getY(), p0.getZ(), p1.getZ(), p1.getX(), Rect::kYZ, m));
			l->add(std::make_shared<Rect>(p0.getY(), p1.getY(), p0.getZ(), p1.getZ(), p0.getX(), Rect::kYZ, m));
			m_list.reset(l);
		}

		virtual bool hit(const Ray& r, float t0, float t1, HitRec& hrec) const override {
			return m_list->hit(r, t0, t1, hrec);
		}

	private:
		vec3 m_p0, m_p1;
		std::unique_ptr<ShapeList> m_list;
	};

	class Translate : public Shape
	{
	public:
		Translate(const ShapePtr& sp, const vec3& displacement)
			: m_shape(sp)
			, m_offset(displacement) {
		}

		virtual bool hit(const Ray& r, float t0, float t1, HitRec& hrec) const override {
			Ray move_r(r.origin() - m_offset, r.direction());
			if (m_shape->hit(move_r, t0, t1, hrec)) {
				hrec.p += m_offset;
				return true;
			}
			else {
				return false;
			}
		}

	private:
		ShapePtr m_shape;
		vec3 m_offset;
	};

	class Rotate : public Shape
	{
	public:
		Rotate(const ShapePtr& sp, const vec3& axis, float angle)
			: m_shape(sp)
			, m_quat(Quat::rotation(radians(angle), axis)) {
		}

		virtual bool hit(const Ray& r, float t0, float t1, HitRec& hrec) const override {
			Quat revq = conj(m_quat);
			vec3 origin = rotate(revq, r.origin());
			vec3 direction = rotate(revq, r.direction());
			Ray rot_r(origin, direction);
			if (m_shape->hit(rot_r, t0, t1, hrec)) {
				hrec.p = rotate(m_quat, hrec.p);
				hrec.n = rotate(m_quat, hrec.n);
				return true;
			}
			else {
				return false;
			}
		}

	private:
		ShapePtr m_shape;
		Quat m_quat;
	};

	//-------------------------------------------------------------------------

	class ShapeBuilder
	{
	public:
		ShapeBuilder() {}
		ShapeBuilder(const ShapePtr& sp)
			: m_ptr(sp) {
		}

		ShapeBuilder& reset(const ShapePtr& sp) {
			m_ptr = sp;
			return *this;
		}

		ShapeBuilder& sphere(const vec3& c, float r, const MaterialPtr& m) {
			m_ptr = std::make_shared<Sphere>(c, r, m);
			return *this;
		}

		ShapeBuilder& rect(float x0, float x1, float y0, float y1, float k, Rect::AxisType axis, const MaterialPtr& m) {
			m_ptr = std::make_shared<Rect>(x0, x1, y0, y1, k, axis, m);
			return *this;
		}

		ShapeBuilder& rectXY(float x0, float x1, float y0, float y1, float k, const MaterialPtr& m) {
			m_ptr = std::make_shared<Rect>(x0, x1, y0, y1, k, Rect::kXY, m);
			return *this;
		}

		ShapeBuilder& rectXZ(float x0, float x1, float y0, float y1, float k, const MaterialPtr& m) {
			m_ptr = std::make_shared<Rect>(x0, x1, y0, y1, k, Rect::kXZ, m);
			return *this;
		}

		ShapeBuilder& rectYZ(float x0, float x1, float y0, float y1, float k, const MaterialPtr& m) {
			m_ptr = std::make_shared<Rect>(x0, x1, y0, y1, k, Rect::kYZ, m);
			return *this;
		}

		ShapeBuilder& rect(const vec3& p0, const vec3& p1, float k, Rect::AxisType axis, const MaterialPtr& m) {
			switch (axis) {
			case Rect::kXY:
				m_ptr = std::make_shared<Rect>(p0.getX(), p1.getX(), p0.getY(), p1.getY(), k, axis, m);
				break;
			case Rect::kXZ:
				m_ptr = std::make_shared<Rect>(p0.getX(), p1.getX(), p0.getZ(), p1.getZ(), k, axis, m);
				break;
			case Rect::kYZ:
				m_ptr = std::make_shared<Rect>(p0.getY(), p1.getY(), p0.getZ(), p1.getZ(), k, axis, m);
				break;
			}
			
			return *this;
		}

		ShapeBuilder& rectXY(const vec3& p0, const vec3& p1, float k, const MaterialPtr& m) {
			return rect(p0, p1, k, Rect::kXY, m);
		}
		ShapeBuilder& rectXZ(const vec3& p0, const vec3& p1, float k, const MaterialPtr& m) {
			return rect(p0, p1, k, Rect::kXZ, m);
		}
		ShapeBuilder& rectYZ(const vec3& p0, const vec3& p1, float k, const MaterialPtr& m) {
			return rect(p0, p1, k, Rect::kYZ, m);
		}

		ShapeBuilder& box(const vec3& p0, const vec3& p1, const MaterialPtr& m) {
			m_ptr = std::make_shared<Box>(p0, p1, m);
			return *this;
		}

		ShapeBuilder& flip() {
			m_ptr = std::make_shared<FlipNormals>(m_ptr);
			return *this;
		}

		ShapeBuilder& translate(const vec3& t) {
			m_ptr = std::make_shared<Translate>(m_ptr, t);
			return *this;
		}

		ShapeBuilder& rotate(const vec3& axis, float angle) {
			m_ptr = std::make_shared<Rotate>(m_ptr, axis, angle);
			return *this;
		}

		const ShapePtr& get() const { return m_ptr; }

	private:
		ShapePtr m_ptr;
	};

	//-------------------------------------------------------------------------
	class Scene
	{
	public:
		Scene(int width, int height, int samples) 
			:m_image(std::make_unique<Image>(width, height))
			,m_backColor(0.1f)
			,m_samples(samples){
		}

		void build() {
			
			m_backColor = vec3(0);

			vec3 lookfrom(278, 278, -800);
			vec3 lookat(278, 278, 0);
			vec3 vup(0, 1, 0);
			float aspect = float(m_image->width()) / float(m_image->height());
			m_camera = std::make_unique<Camera>(lookfrom, lookat, vup, 40, aspect);
			
			
			/*
			vec3 w(-2.0f, -1.0f, -1.0f);
			vec3 u(4.0f, 0.0f, 0.0f);
			vec3 v(0.0f, 2.0f, 0.0f);
			m_camera = std::make_unique<Camera>(u, v, w);
			*/

			MaterialPtr red = std::make_shared<Lambertian>(std::make_shared<ColorTexture>(vec3(0.65f, 0.05f, 0.05f)));
			MaterialPtr white = std::make_shared<Lambertian>(std::make_shared<ColorTexture>(vec3(0.73f)));
			MaterialPtr green = std::make_shared<Lambertian>(std::make_shared<ColorTexture>(vec3(0.12f, 0.45f, 0.15f)));
			MaterialPtr light = std::make_shared<DiffuseLight>(std::make_shared<ColorTexture>(vec3(15.0f)));

			ShapeList* world = new ShapeList();
			
			
			ShapeBuilder builder;
			world->add(builder.rectYZ(0, 555, 0, 555, 555, green).flip().get());
			world->add(builder.rectYZ(0, 555, 0, 555, 0, red).get());
			world->add(builder.rectXZ(213, 343, 227, 332, 554, light).get());
			world->add(builder.rectXZ(0, 555, 0, 555, 555, white).flip().get());
			world->add(builder.rectXZ(0, 555, 0, 555, 0, white).get());
			world->add(builder.rectXY(0, 555, 0, 555, 555, white).flip().get());
			world->add(builder.box(vec3(0), vec3(165), white).rotate(vec3::yAxis(), -18).translate(vec3(130, 0, 65)).get());
			world->add(builder.box(vec3(0), vec3(165, 330, 165), white).rotate(vec3::yAxis(), 15).translate(vec3(265, 0, 295)).get());
			

			/*world->add(std::make_shared<Rect>(0, 555, 0, 555, 555, Rect::kYZ, green));
			world->add(std::make_shared<Rect>(0, 555, 0, 555, 0, Rect::kYZ, red));
			world->add(std::make_shared<Rect>(213, 343, 227, 332, 554, Rect::kXZ, light));
			world->add(std::make_shared<Rect>(0, 555, 0, 555, 555, Rect::kXZ, white));
			world->add(std::make_shared<Rect>(0, 555, 0, 555, 0, Rect::kXZ, white));
			world->add(std::make_shared<Rect>(0, 555, 0, 555, 555, Rect::kXY, white));

			world->add(std::make_shared<Box>(vec3(130, 0, 65), vec3(295, 165, 230), white));
			world->add(std::make_shared<Box>(vec3(265, 0, 295), vec3(430, 330, 460), white));*/

			//world->add(std::make_shared<Sphere>(vec3(0.6, 0, -1), 0.5f, std::make_shared<Lambertian>(std::make_shared<ImageTexture>("./brick_diffuse.jpg"))));
			//world->add(std::make_shared<Sphere>(vec3(-0.6, 0, -1), 0.5f, std::make_shared<Metal>(std::make_shared<ColorTexture>(vec3(0.8f, 0.8f, 0.8f)), 0.4f)));
			//world->add(std::make_shared<Sphere>(vec3(0, 2, 0), 2, std::make_shared<Lambertian>(std::make_shared<ColorTexture>(vec3(0.5f, 0.5f, 0.5f)))));
			//world->add(std::make_shared<Sphere>(vec3(0, -1000, 0), 1000, std::make_shared<Lambertian>(std::make_shared<ColorTexture>(vec3(0.8f, 0.8f, 0.8f)))));
			//world->add(std::make_shared<Rect>(3, 5, 1, 3, -2, Rect::kXY, std::make_shared<DiffuseLight>(std::make_shared<ColorTexture>(vec3(4)))));
			//world->add(std::make_shared<Sphere>(vec3(0, -100.5, -1), 100, std::make_shared<Lambertian>(std::make_shared<CheckerTexture>(std::make_shared<ColorTexture>(vec3(0.8f, 0.8f, 0.0f)), std::make_shared<ColorTexture>(vec3(0.8f, 0.2f, 0.0f)), 10.0f))));
			m_world.reset(world);

		}

		float hit_sphere(const vec3& center, float radius, const rayt::Ray& r) const {
			vec3 oc = r.origin() - center;
			float a = dot(r.direction(), r.direction());
			float b = 2.0f * dot(r.direction(), oc);
			float c = dot(oc, oc) - pow2(radius);
			float D = pow2(b) - 4 * a*c;
			if (D < 0) {
				return -1.0f;
			}
			else {
				return (-b - sqrtf(D)) / (2.0f*a);
			}
		}


		vec3 color(const rayt::Ray& r, const Shape* world, int depth) {
			HitRec hrec;
			if (world->hit(r, 0.001f, FLT_MAX, hrec)) {
				vec3 emitted = hrec.mat->emitted(r, hrec);
				ScatterRec srec;
				if (depth < MAX_DEPTH && hrec.mat->scatter(r, hrec, srec)) {
					return emitted + mulPerElem(srec.albedo, color(srec.ray, world, depth+1));
				}
				else {
					return emitted;
				}
			}
			return background(r.direction());
		}

		vec3 background(const vec3& d) const {
			return m_backColor;
		}

		vec3 backgroundSky(const vec3& d) const {
			vec3 v = normalize(d);
			float t = 0.5f * (v.getY() + 1.0f);
			return lerp(t, vec3(1), vec3(0.5f, 0.7f, 1.0f));
		}

		void render() {
			build();

			int nx = m_image->width();
			int ny = m_image->height();

#pragma omp parallel for schedule(dynamic, 1) num_threads(NUM_THREAD)
			for (int j = 0; j < ny; ++j) {
				std::cerr << "Rendering (y = " << j << ") " << (100.0 * j / (ny - 1)) << "%" << std::endl;
				for (int i = 0; i < nx; ++i) {
					vec3 c(0);
					for (int s = 0; s < m_samples; ++s) {
						float u = float(i + drand48()) / float(nx);
						float v = float(j + drand48()) / float(ny);
						Ray r = m_camera->getRay(u, v);
						c += color(r, m_world.get(), 0);
					}
					c /= m_samples;
					m_image->write(i, (ny - j - 1), c.getX(), c.getY(), c.getZ());
				}
			}

			stbi_write_jpg("render209.jpg", nx, ny, sizeof(rayt::Image::rgb), m_image->pixels(), 100);
		}

	private:
		std::unique_ptr<Camera> m_camera;
		std::unique_ptr<Image> m_image;
		std::unique_ptr<Shape> m_world;
		vec3 m_backColor;
		int m_samples;
	};
}


int main()
{
	clock_t start = clock();
	int nx = 1280;
	int ny = 760;
	int ns = 50;

	std::unique_ptr<rayt::Scene> scene(std::make_unique<rayt::Scene>(nx, ny, ns));
	scene->render();
	clock_t end = clock();
	std::cerr << start - end << std::endl;

	int hoge;
	std::cin >> hoge;
	return 0;
}