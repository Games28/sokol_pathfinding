#define SOKOL_GLCORE
#include "sokol_engine.h"
#include "sokol/sokol_gfx.h"
#include "sokol/sokol_glue.h"
#include <iostream>

#include "shd.glsl.h"

#include "math/v3d.h"
#include "v2d.h"
#include "math/mat4.h"

#include "mesh.h"
#include "Camera.h"
#include "AABB3.h"
#include "AABB.h"
#include "poisson_disc.h"
#include "Graph.h"
#include "Triangulate.h"

//for time
#include <ctime>
#include "Node.h"

#include "texture_utils.h"
#include "Object.h"




//y p => x y z
//0 0 => 0 0 1
static cmn::vf3d polar3D(float yaw, float pitch) {
	return {
		std::sin(yaw)*std::cos(pitch),
		std::sin(pitch),
		std::cos(yaw)*std::cos(pitch)
	};
}




struct Light
{
	cmn::vf3d pos;
	sg_color col;

};

struct Demo : SokolEngine {
	sg_pipeline default_pip{};
	sg_pipeline line_pip{};
	sg_pipeline terrain_pip{};
	
	Graph graph;
	sg_sampler sampler{};
	bool render_outlines = false;

	mat4 cam_view_proj;

	std::vector<Light> lights;
	Light* mainlight;

	std::vector<Object> objects;
	std::vector<Object> billboard_nodes;
	
	const std::vector<std::string> Structurefilenames{
		"assets/models/deserttest.txt",
		"assets/models/tathouse1.txt",
		"assets/models/tatooinehouse1.txt",
	};

	const std::vector<std::string> texturefilenames
	{
		"assets/poust_1.png",
		"assets/GroundSand.png",
		"assets/T_pillar.png"
	};

	sg_view tex_blank{};
	sg_view tex_uv{};

	sg_view gui_image{};

	sg_pass_action display_pass_action{};

	Object platform;



	static inline uint32_t xorshift32(void) {
		static uint32_t x = 0x12345678;
		x ^= x << 13;
		x ^= x >> 17;
		x ^= x << 5;
		return x;
	}

#pragma region SETUP HELPERS
	void setupEnvironment() {
		sg_desc desc{};
		desc.environment=sglue_environment();
		sg_setup(desc);
	}

	void setupTerrain()
	{
		sg_pipeline_desc pipeline_desc{};
		pipeline_desc.layout.attrs[ATTR_terrain_i_pos].format = SG_VERTEXFORMAT_FLOAT3;
		pipeline_desc.layout.attrs[ATTR_terrain_i_norm].format = SG_VERTEXFORMAT_FLOAT3;
		pipeline_desc.layout.attrs[ATTR_terrain_i_uv].format = SG_VERTEXFORMAT_FLOAT2;
		pipeline_desc.shader = sg_make_shader(default_shader_desc(sg_query_backend()));
		pipeline_desc.index_type = SG_INDEXTYPE_UINT32;
		pipeline_desc.cull_mode = SG_CULLMODE_FRONT;
		pipeline_desc.depth.write_enabled = true;
		pipeline_desc.depth.compare = SG_COMPAREFUNC_LESS_EQUAL;
		terrain_pip = sg_make_pipeline(pipeline_desc);
	}

	void setupLights()
	{
		//white
		lights.push_back({ {-1,3,1},{1,1,1,1} });
		mainlight = &lights.back();
		
	}
	
	//primitive textures to debug with
	void setupTextures() {
		tex_blank=makeBlankTexture();
		tex_uv=makeUVTexture(512, 512);
	}

	//if texture loading fails, default to uv tex.
	sg_view getTexture(const std::string& filename) {
		sg_view tex;
		auto status=makeTextureFromFile(tex, filename);
		if(!status.valid) tex=tex_uv;
		return tex;
	}

	void setupSampler() {
		sg_sampler_desc sampler_desc{};
		sampler=sg_make_sampler(sampler_desc);
	}

	void setupLinePipeline()
	{
		sg_pipeline_desc pip_desc{};
		pip_desc.layout.attrs[ATTR_line_v_pos].format = SG_VERTEXFORMAT_FLOAT3;
		pip_desc.layout.attrs[ATTR_line_v_col].format = SG_VERTEXFORMAT_FLOAT4;
		pip_desc.shader = sg_make_shader(line_shader_desc(sg_query_backend()));
		pip_desc.primitive_type = SG_PRIMITIVETYPE_LINES;
		pip_desc.index_type = SG_INDEXTYPE_UINT32;
		pip_desc.depth.write_enabled = true;
		pip_desc.depth.compare = SG_COMPAREFUNC_LESS_EQUAL;
		line_pip = sg_make_pipeline(pip_desc);
	}

	void setupObjects()
	{
		std::vector<cmn::vf3d> poses
		{
			{ 0,-2,0 },
			{ 5,-2,-7 },
			{ -5,-2,3 }
		};

		std::vector<cmn::vf3d> scales
		{
			{2,2,2},
			{0.5, 0.5f,0.5f},
			{0.5, 0.5f,0.5f}
		};

		for (int i = 0; i < Structurefilenames.size(); i++)
		{
			Mesh m;
			auto status = Mesh::loadFromOBJ(m, Structurefilenames[i]);
			if (!status.valid) m = Mesh::makeCube();
			objects.push_back(Object(m, getTexture(texturefilenames[i])));
			objects[i].scale = scales[i];
			objects[i].translation = poses[i];
			objects[i].updateMatrixes();
		}
	
	}

	void setupNodes()
	{
		//randomly sample points on xz plane
		Object terrian = objects[0];

		AABB3 bounds = terrian.getAABB();
		auto xz_pts = poissonDiscSample({ {bounds.min.x,bounds.min.z}, {bounds.max.x,bounds.max.z} }, 2);

		//project pts on to terrain
		std::unordered_map<cmn::vf2d*, Node*> xz2way;
		for (auto& p : xz_pts)
		{
			cmn::vf3d orig(p.x, bounds.min.y - .1f, p.y);
			cmn::vf3d dir(0, 1, 0);
			float dist = terrian.intersectRay(orig,dir);
			graph.nodes.push_back(new Node(orig + (.2f + dist) * dir));
			xz2way[&p] = graph.nodes.back();

		}

		//trangulate and add links
		auto tris =delaunay::triangulate(xz_pts);
		auto edges = delaunay::extractEdges(tris);
		
		for (const auto& e : edges)
		{
			auto a = xz2way[&xz_pts[e.p[0]]];
			auto b = xz2way[&xz_pts[e.p[1]]];
			graph.addLink(a, b);
			graph.addLink(b, a);
		}

		//remove any nodes in way of obstacle
		for (auto it = graph.nodes.begin(); it != graph.nodes.end();)
		{
			auto& n = *it;
			//check if inside any meshes
			bool blocked = false;
			for (int i = 1; i < objects.size(); i++)
			{
				if (objects[i].contains(n->pos))
				{
					blocked = true;
					break;
				}
			}

			if (blocked)
			{
				for (auto& o : graph.nodes)
				{
					auto oit = std::find(o->links.begin(), o->links.end(), n);
					if (oit != o->links.end()) o->links.erase(oit);
				}

				delete n;
				it = graph.nodes.erase(it);

			}
			else
			{
				it++;
			}

		}


	}

	void setupPlatform() {
		Object obj;
		Mesh& m=obj.mesh;
		m=Mesh::makeCube();

		obj.tex = getTexture("assets/poust_1.png");
		
		obj.scale={10, .25f, 10};
		obj.translation={0, -1, 0};
		obj.updateMatrixes();
		objects.push_back(obj);
	}

	void setupBillboard() {
		Object obj;
		Mesh& m=obj.mesh;
		m.verts={
			{{-.5f, .5f, 0}, {0, 0, 1}, {0, 0}},//tl
			{{.5f, .5f, 0}, {0, 0, 1}, {1, 0}},//tr
			{{-.5f, -.5f, 0}, {0, 0, 1}, {0, 1}},//bl
			{{.5f, -.5f, 0}, {0, 0, 1}, {1, 1}}//br
		};
		m.tris={
			{0, 2, 1},
			{1, 2, 3}
		};
		m.updateVertexBuffer();
		m.updateIndexBuffer();

		objects.push_back(Object(m, getTexture("assets/spritesheet.png")));
		objects[objects.size() - 1].translation = { 0,1,0 };
		objects[objects.size() - 1].updateMatrixes();
		objects[objects.size() - 1].num_x = 4; objects[objects.size() - 1].num_y = 4;
		objects[objects.size() - 1].num_ttl = objects[objects.size() - 1].num_x * objects[objects.size() - 1].num_y;
		objects[objects.size() - 1].isbillboard = true;
	
	}

	void setupNodeBillboards()
	{
		int count = 0;
		for (const auto& n : graph.nodes)
		{
			
			Object obj;
			Mesh& m = obj.mesh;
			m.verts = {
				{{-.5f, .5f, 0}, {0, 0, 1}, {0, 0}},//tl
				{{.5f, .5f, 0}, {0, 0, 1}, {1, 0}},//tr
				{{-.5f, -.5f, 0}, {0, 0, 1}, {0, 1}},//bl
				{{.5f, -.5f, 0}, {0, 0, 1}, {1, 1}}//br
			};
			m.tris = {
				{0, 2, 1},
				{1, 2, 3}
			};
			m.updateVertexBuffer();
			m.updateIndexBuffer();

			billboard_nodes.push_back(Object(m, tex_uv));
			billboard_nodes[count].scale = { 0.4f,0.4f,0.4f };
			billboard_nodes[count].translation = n->pos;
			billboard_nodes[count].updateMatrixes();
			billboard_nodes[count].num_x = 1;
			billboard_nodes[count].num_y = 1;
			billboard_nodes[count].num_ttl = billboard_nodes[count].num_x * billboard_nodes[count].num_y;

			count++;

		}
	}

	//clear to bluish
	void setupDisplayPassAction() {
		display_pass_action.colors[0].load_action=SG_LOADACTION_CLEAR;
		display_pass_action.colors[0].clear_value={.25f, .45f, .65f, 1.f};
	}

	void setupDefaultPipeline() {
		sg_pipeline_desc pipeline_desc{};
		pipeline_desc.layout.attrs[ATTR_default_v_pos].format=SG_VERTEXFORMAT_FLOAT3;
		pipeline_desc.layout.attrs[ATTR_default_v_norm].format=SG_VERTEXFORMAT_FLOAT3;
		pipeline_desc.layout.attrs[ATTR_default_v_uv].format=SG_VERTEXFORMAT_FLOAT2;
		pipeline_desc.shader=sg_make_shader(default_shader_desc(sg_query_backend()));
		pipeline_desc.index_type=SG_INDEXTYPE_UINT32;
		pipeline_desc.cull_mode=SG_CULLMODE_FRONT;
		pipeline_desc.depth.write_enabled=true;
		pipeline_desc.depth.compare=SG_COMPAREFUNC_LESS_EQUAL;
		default_pip=sg_make_pipeline(pipeline_desc);
	}
#pragma endregion

	void userCreate() override {
		setupEnvironment();

		setupLinePipeline();

		setupTerrain();

		

		setupTextures();

		setupSampler();

		setupLights();
		
		setupObjects();

		
		//setup nodes
		setupNodes();
		setupNodeBillboards();


		setupBillboard();
		

		
		setupDisplayPassAction();

		setupDefaultPipeline();
	}

#pragma region UPDATE HELPERS
	


	void handleCameraLooking(float dt) {
		//left/right
		if(getKey(SAPP_KEYCODE_LEFT).held) cam.yaw+=dt;
		if(getKey(SAPP_KEYCODE_RIGHT).held) cam.yaw-=dt;

		//up/down
		if(getKey(SAPP_KEYCODE_UP).held) cam.pitch+=dt;
		if(getKey(SAPP_KEYCODE_DOWN).held) cam.pitch-=dt;

		//clamp camera pitch
		if(cam.pitch>Pi/2) cam.pitch=Pi/2-.001f;
		if(cam.pitch<-Pi/2) cam.pitch=.001f-Pi/2;

		
	}

	void handleCameraMovement(float dt) {

		
		
		//move up, down
		if (getKey(SAPP_KEYCODE_SPACE).held) cam.pos.y += 4.f * dt;
		if (getKey(SAPP_KEYCODE_LEFT_SHIFT).held) cam.pos.y -= 4.f * dt;
		

		//move forward, backward
		cmn::vf3d fb_dir(std::sin(cam.yaw), 0, std::cos(cam.yaw));
		if(getKey(SAPP_KEYCODE_W).held) cam.pos+=5.f*dt*fb_dir;
		if(getKey(SAPP_KEYCODE_S).held) cam.pos-=3.f*dt*fb_dir;

		//move left, right
		cmn::vf3d lr_dir(fb_dir.z, 0, -fb_dir.x);
		if(getKey(SAPP_KEYCODE_A).held) cam.pos+=4.f*dt*lr_dir;
		if(getKey(SAPP_KEYCODE_D).held) cam.pos-=4.f*dt*lr_dir;

	}

	void handleUserInput(float dt) {
		handleCameraLooking(dt);
		//polar to cartesian
		cam.dir=polar3D(cam.yaw, cam.pitch);
		if (getKey(SAPP_KEYCODE_R).held) mainlight->pos = cam.pos;
		//toggle shape outlines
		if (getKey(SAPP_KEYCODE_O).pressed) render_outlines ^= true;

		handleCameraMovement(dt);
	}

	void updateNodeBillboard(Object& obj, float dt)
	{
		//move with player 
		cmn::vf3d eye_pos = obj.translation;
		cmn::vf3d target = cam.pos;

		cmn::vf3d y_axis(0, 1, 0);
		cmn::vf3d z_axis = (target - eye_pos).norm();
		cmn::vf3d x_axis = y_axis.cross(z_axis).norm();
		y_axis = z_axis.cross(x_axis);

		//slightly different than makeLookAt.
		mat4& m = obj.model;
		m(0, 0) = x_axis.x, m(0, 1) = y_axis.x, m(0, 2) = z_axis.x, m(0, 3) = eye_pos.x;
		m(1, 0) = x_axis.y, m(1, 1) = y_axis.y, m(1, 2) = z_axis.y, m(1, 3) = eye_pos.y;
		m(2, 0) = x_axis.z, m(2, 1) = y_axis.z, m(2, 2) = z_axis.z, m(2, 3) = eye_pos.z;
		m(3, 3) = 1;
	}

	//make billboard always point at camera.
	void updateBillboard(Object& obj, float dt) {
		//move with player 
		cmn::vf3d eye_pos= obj.translation;
		cmn::vf3d target=cam.pos;

		cmn::vf3d y_axis(0, 1, 0);
		cmn::vf3d z_axis=(target-eye_pos).norm();
		cmn::vf3d x_axis=y_axis.cross(z_axis).norm();
		y_axis=z_axis.cross(x_axis);
		
		//slightly different than makeLookAt.
		mat4& m= obj.model;
		m(0, 0)=x_axis.x, m(0, 1)=y_axis.x, m(0, 2)=z_axis.x, m(0, 3)=eye_pos.x;
		m(1, 0)=x_axis.y, m(1, 1)=y_axis.y, m(1, 2)=z_axis.y, m(1, 3)=eye_pos.y;
		m(2, 0)=x_axis.z, m(2, 1)=y_axis.z, m(2, 2)=z_axis.z, m(2, 3)=eye_pos.z;
		m(3, 3)=1;
		
		float angle = atan2f(z_axis.z, z_axis.x);
		//
		//int i = 0;
		//
		if (angle < -0.70 && angle > -2.35 )
		{
			obj.anim = 1; //front
		}
		if (angle < -2.35 && angle < 2.35)
		{
			obj.anim = 4; //left
		}
		if (angle > -0.70 && angle < 0.70)
		{
			obj.anim = 8; //right
		}
		if (angle > 0.70 && angle < 2.35) 
		{
			obj.anim = 12; //back
		}
		//obj.anim_timer-=dt;
		//if(obj.anim_timer<0) {
		//	obj.anim_timer+=.5f;
		//
		//	//increment animation index and wrap
		//	obj.anim++;
		//	obj.anim%=obj.num_ttl;
		//}
	}

	

	
	

#pragma endregion

	void updateCameraMatrixes() {
		mat4 look_at = mat4::makeLookAt(cam.pos, cam.pos + cam.dir, { 0, 1, 0 });
		cam.view = mat4::inverse(look_at);

		//cam proj can change with window resize
		float asp = sapp_widthf() / sapp_heightf();
		cam.proj = mat4::makePerspective(90, asp, .001f, 1000.f);

		cam.view_proj = mat4::mul(cam.proj, cam.view);
	}


	void userUpdate(float dt) {
		
		handleUserInput(dt);
	
		updateCameraMatrixes();

		for (auto& obj : objects)
		{
			if (obj.isbillboard)
			{
				updateBillboard(obj, dt);
				
			}
			
		}

		//update billboard nodes
		for (auto& obj : billboard_nodes)
		{
			updateNodeBillboard(obj, dt);
		}
		
		
	}

#pragma region RENDER HELPERS

	 


	void renderObjects(Object& obj) {
		sg_bindings bind{};
		bind.vertex_buffers[0]=obj.mesh.vbuf;
		bind.index_buffer= obj.mesh.ibuf;
		bind.samplers[SMP_default_smp]=sampler;
		bind.views[VIEW_default_tex] = obj.tex;
		sg_apply_bindings(bind);

		//pass transformation matrix
		mat4 mvp=mat4::mul(cam.view_proj, obj.model);
		vs_params_t vs_params{};
		std::memcpy(vs_params.u_model, obj.model.m, sizeof(vs_params.u_model));
		std::memcpy(vs_params.u_mvp, mvp.m, sizeof(mvp.m));
		sg_apply_uniforms(UB_vs_params, SG_RANGE(vs_params));

		//render entire texture.
		//fs_params_t fs_params{};
		//lighting test
		fs_params_t fs_params{};
		{

			fs_params.u_num_lights = lights.size();
			int idx = 0;
			for (const auto& l : lights)
			{
				fs_params.u_light_pos[idx][0] = l.pos.x;
				fs_params.u_light_pos[idx][1] = l.pos.y;
				fs_params.u_light_pos[idx][2] = l.pos.z;
				fs_params.u_light_col[idx][0] = l.col.r;
				fs_params.u_light_col[idx][1] = l.col.g;
				fs_params.u_light_col[idx][2] = l.col.b;
				idx++;
			}
		}

		fs_params.u_view_pos[0] = cam.pos.x;
		fs_params.u_view_pos[1] = cam.pos.y;
		fs_params.u_view_pos[2] = cam.pos.z;
		//sg_apply_uniforms(UB_fs_params, SG_RANGE(fs_params));


		fs_params.u_tl[0]=0, fs_params.u_tl[1]=0;
		fs_params.u_br[0]=1, fs_params.u_br[1]=1;
		sg_apply_uniforms(UB_fs_params, SG_RANGE(fs_params));

		sg_draw(0, 3* obj.mesh.tris.size(), 1);
	}
	
	void renderBillboard(Object& obj) {
		sg_bindings bind{};
		bind.vertex_buffers[0]= obj.mesh.vbuf;
		bind.index_buffer= obj.mesh.ibuf;
		bind.samplers[SMP_default_smp]=sampler;
		bind.views[VIEW_default_tex]= obj.tex;
		sg_apply_bindings(bind);

		//pass transformation matrix
		mat4 mvp=mat4::mul(cam.view_proj, obj.model);
		vs_params_t vs_params{};
		std::memcpy(vs_params.u_mvp, mvp.m, sizeof(mvp.m));
		sg_apply_uniforms(UB_vs_params, SG_RANGE(vs_params));

		//which region of texture to sample?

		fs_params_t fs_params{};
		int row= obj.anim/ obj.num_x;
		int col= obj.anim% obj.num_x;
		float u_left=col/float(obj.num_x);
		float u_right=(1+col)/float(obj.num_x);
		float v_top=row/float(obj.num_y);
		float v_btm=(1+row)/float(obj.num_y);
		fs_params.u_tl[0]=u_left;
		fs_params.u_tl[1]=v_top;
		fs_params.u_br[0]=u_right;
		fs_params.u_br[1]=v_btm;
		sg_apply_uniforms(UB_fs_params, SG_RANGE(fs_params));

		sg_draw(0, 3* obj.mesh.tris.size(), 1);
	}

	void renderObjectOutlines(const Object& obj)
	{
	
			sg_apply_pipeline(line_pip);

			sg_bindings bind{};
			bind.vertex_buffers[0] = obj.linemesh.vbuf;
			bind.index_buffer = obj.linemesh.ibuf;
			sg_apply_bindings(bind);

			vs_line_params_t vs_line_params{};
			mat4 mvp = mat4::mul(cam.view_proj, obj.model);
			std::memcpy(vs_line_params.u_mvp, mvp.m, sizeof(vs_line_params.u_mvp));
			sg_apply_uniforms(UB_vs_line_params, SG_RANGE(vs_line_params));

			sg_draw(0, 2 * obj.linemesh.lines.size(), 1);
		
	}
	

#pragma endregion
	
	void userRender() {
		sg_pass pass{};
		pass.action=display_pass_action;
		pass.swapchain=sglue_swapchain();
		sg_begin_pass(pass);

		//camera transformation matrix
		mat4 look_at=mat4::makeLookAt(cam.pos, cam.pos+cam.dir, {0, 1, 0});
		
		mat4 cam_view=mat4::inverse(look_at);


		//perspective
		mat4 cam_proj=mat4::makePerspective(90.f, sapp_widthf()/sapp_heightf(), .001f, 1000);
		
		//premultiply transform
		// cam_view_proj=mat4::mul(cam_proj, cam_view);

		sg_apply_pipeline(default_pip);

		
		

		for (auto& obj : objects)
		{
			if (render_outlines)
			{
				renderObjectOutlines(obj);
			}
			else
			{
              
				
					renderObjects(obj);
				

			}
			
		}

		//render billboards nodes
		for (auto& obj : billboard_nodes)
		{
			renderObjects(obj);
		}
		
		sg_end_pass();
		
		sg_commit();
	}
};