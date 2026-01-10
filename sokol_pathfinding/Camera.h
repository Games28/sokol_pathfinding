struct
{
	cmn::vf3d pos{ 0,2,2 };
	cmn::vf3d dir;
	float yaw = 0;
	float pitch = 0;
	mat4 proj, view;
	mat4 view_proj;
}cam;
