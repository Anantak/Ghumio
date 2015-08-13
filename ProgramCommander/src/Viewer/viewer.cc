/** Viewer
 * Implementation of Viewer
 */

// OpenGL and QGLViewer includes
#include "Viewer/viewer.h"

// Eigen library


namespace anantak {
	
// Options
Viewer::Options::Options(float radius) {
	float scale = radius / 5.;
	
	position_sphere_size = 0.05*scale;
	rotation_axes_line_width = 3;
	rotation_axes_line_length = 1.0*scale;
	ellipsoid_line_width = 1;
	num_points_on_circle = 16;
	
	scene_width = 5.*scale;		// meters
	scene_length = 5.*scale;	// meters
	
	velocity_arrow_length_ratio = 0.1*scale;    // m/s per meter
	velocity_arrow_radius_ratio = 0.3*scale;
	
	target_height = .25*scale;
	target_width = .33*scale;
	target_depth = .01*scale;
	
	meters_per_pixel = 1./2000.;
	
	timer_delay = 50000;		// in microseconds
}
	
// Constructor takes in a scene
Viewer::Viewer(anantak::Scene* s, float scale = 1.) :
		options_(scale), scene_(s), dynamic_scene_(nullptr) {
	if (!scene_) {LOG(FATAL) << "Scene is NULL. Can not continue";}
	Initiate();
}

// Constructor takes in a scene
Viewer::Viewer(anantak::DynamicScene* s, float scale = 1.) :
		options_(scale), scene_(nullptr), dynamic_scene_(s) {
	if (!dynamic_scene_) {LOG(FATAL) << "DynamicScene is NULL. Can not continue";}
	Initiate();
}

bool Viewer::Initiate() {
	// Setup circles
	float dtheta = float(kPi_2) / float(options_.num_points_on_circle);
	xy_circle_points_.resize(3, options_.num_points_on_circle);
	yz_circle_points_.resize(3, options_.num_points_on_circle);
	zx_circle_points_.resize(3, options_.num_points_on_circle);
	for (int i=0; i<options_.num_points_on_circle; i++) {
		float theta = dtheta*float(i);
		xy_circle_points_(0,i) = std::cos(theta);
		xy_circle_points_(1,i) = std::sin(theta);
		xy_circle_points_(2,i) = 0.;
		yz_circle_points_(0,i) = 0.;
		yz_circle_points_(1,i) = std::sin(theta);
		yz_circle_points_(2,i) = std::cos(theta);
		zx_circle_points_(0,i) = std::cos(theta);
		zx_circle_points_(1,i) = 0.;
		zx_circle_points_(2,i) = std::sin(theta);
	}
	//// Timer
	//connect(&viewer_timer_, SIGNAL(timeout()), this, SLOT(updateGL()));
	//viewer_timer_.start(options_.timer_delay/1000);
}

// Destructor - all should self destruct
Viewer::~Viewer() {}

// Update
//void Viewer::updateGL() {
//	VLOG(1) << "Update called";
//	if (dynamic_scene_) {
//		dynamic_scene_->Update();
//	}
//	// Redraw the scene
//	draw();
//}

void Viewer::init() {
	// Set scene
	setSceneRadius(std::max(options_.scene_width, options_.scene_length));   // scene radius in GL units
	VLOG(1) << "Scene radius = " << sceneRadius();
	setSceneCenter(qglviewer::Vec(0,0,0)); 		// scene center is a origin
	camera()->showEntireScene();
	
  // Restore previous viewer state.
  restoreStateFromFile();
	
	// Opens help window
  //help();
	
	// Start animation
	startAnimation();
}

void Viewer::animate() {
	VLOG(3) << "Update called";
	if (dynamic_scene_) {
		dynamic_scene_->Update();
	}
}

void Viewer::draw() {
	
	// Draw the static scene objects
	if (scene_) {
		for (auto it=scene_->scene_poses_.begin(); it!=scene_->scene_poses_.end(); it++) {
			DrawPoseState(*it);
			DrawVelocityState(*it);
		}
	}
	
	// Draw the dynamic scene objects
	if (dynamic_scene_) {
		DrawCamera(dynamic_scene_->camera_display_);
		DrawPoseState(dynamic_scene_->target_pose_display_);
		DrawVelocityState(dynamic_scene_->target_pose_display_);
	}
	
}

// Draws a spiral
void Viewer::DrawSpiral() {
  const float nbSteps = 200.0;

  glBegin(GL_QUAD_STRIP);
  for (int i=0; i<nbSteps; ++i) {
	  const float ratio = i/nbSteps;
	  const float angle = 21.0*ratio;
	  const float c = cos(angle);
	  const float s = sin(angle);
	  const float r1 = 1.0 - 0.8f*ratio;
	  const float r2 = 0.8f - 0.8f*ratio;
	  const float alt = ratio - 0.5f;
	  const float nor = 0.5f;
	  const float up = sqrt(1.0-nor*nor);
	  glColor3f(1.0-ratio, 0.2f , ratio);
	  glNormal3f(nor*c, up, nor*s);
	  glVertex3f(r1*c, alt, r1*s);
	  glVertex3f(r2*c, alt+0.05f, r2*s);
	}
  glEnd();	
}

// Draws an Ellipsoid
bool Viewer::DrawEllipsoid() {
	
	//glEnable(GL_BLEND);
	//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glLineWidth(1);
	
	//GLfloat  mat[16];	
	GLUquadricObj	*obj = gluNewQuadric();
	if (!obj) {
		return false;
	}
	
	glDisable(GL_LIGHTING);  // Disable lights when drawing lines
	gluQuadricDrawStyle(obj, false ? GLU_FILL : GLU_LINE);
	//glPushMatrix();
	//glMultMatrixf(mat);
	glScalef(.1,.2,.3);
	gluSphere(obj, 1, 10, 10);
	glPopMatrix();
	gluDeleteQuadric(obj);

	//glDisable(GL_BLEND);
	glEnable(GL_LIGHTING);
	
	return true;
}

bool Viewer::DrawArrow(float length, float radius, int nbSubdivisions) {
	
	GLUquadric* quadric = gluNewQuadric();
	if (radius < 0.0)
		radius = 0.05 * length;
	const float head = 2.5*(radius / length) + 0.1;
	const float coneRadiusCoef = 4.0 - 5.0 * head;

	//glDisable(GL_LIGHTING);  // Disable lights when drawing lines
	gluQuadricDrawStyle(quadric, GLU_FILL);
	
	gluCylinder(quadric, radius, radius, length * (1.0 - head/coneRadiusCoef), nbSubdivisions, 1);
	glTranslated(0.0, 0.0, length * (1.0 - head));
	gluCylinder(quadric, coneRadiusCoef * radius, 0.0, head * length, nbSubdivisions, 1);
	glTranslated(0.0, 0.0, -length * (1.0 - head));
	gluDeleteQuadric(quadric);

	//glEnable(GL_LIGHTING);
	
	return true;
}

bool Viewer::DrawLine(float length = 1.) {
	glDisable(GL_LIGHTING);  // Disable lights when drawing lines
	glLineWidth(options_.rotation_axes_line_width);
	glBegin(GL_LINES);
	glVertex3f(0.,0.,0.);
	glVertex3f(length,0.,0.);
	glEnd();
	glEnable(GL_LIGHTING);
}

bool Viewer::DrawTarget(float height = 1., float width = 1., float depth = 0.1) {
	glDisable(GL_LIGHTING);  // Disable lights when drawing lines
	glLineWidth(options_.rotation_axes_line_width);
	glColor4f(1.,1.,0.,1.);
	glBegin(GL_LINE_LOOP);
	glVertex3f( width,  height, 0.);
	glVertex3f( width, -height, 0.);
	glVertex3f(-width, -height, 0.);
	glVertex3f(-width,  height, 0.);
	glEnd();
	glEnable(GL_LIGHTING);
}



// Draw a circle
bool Viewer::DrawCircleXY() {
	glBegin(GL_LINE_LOOP);
	for (int i=0; i<options_.num_points_on_circle; i++) {
		glVertex3f(xy_circle_points_(0,i), xy_circle_points_(1,i), xy_circle_points_(2,i));
	}
	glEnd();
	return true;
}
bool Viewer::DrawCircleYZ() {
	glBegin(GL_LINE_LOOP);
	for (int i=0; i<options_.num_points_on_circle; i++) {
		glVertex3f(yz_circle_points_(0,i), yz_circle_points_(1,i), yz_circle_points_(2,i));
	}
	glEnd();
	return true;
}
bool Viewer::DrawCircleZX() {
	glBegin(GL_LINE_LOOP);
	for (int i=0; i<options_.num_points_on_circle; i++) {
		glVertex3f(zx_circle_points_(0,i), zx_circle_points_(1,i), zx_circle_points_(2,i));
	}
	glEnd();
	return true;
}

// Draw a pose state
bool Viewer::DrawPoseState(const anantak::PoseStateDisplay& pose) {
	
	if (!pose.has_pose_) return false;
	
	//glEnable(GL_BLEND);
	//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	
	// A homogeneous transformation matrix, in this order:
	// 		0  4  8  12
	//    1  5  9  13
	//    2  6  10 14
	//    3  7  11 15
	const Eigen::Matrix3d& rotn = pose.R_;
	const Eigen::Vector3d& posn = pose.p_;
	GLfloat mat[16];
	mat[3] = mat[7] = mat[11] = 0; mat[15] = 1;
	mat[12] = posn(0); mat[13] = posn(1); mat[14] = posn(2);	// New position
	// We do a transpose here as rotation stored is conjugate
	mat[0] = rotn(0,0); mat[4] = rotn(1,0); mat[8] = rotn(2,0);	// New X-axis
	mat[1] = rotn(0,1); mat[5] = rotn(1,1); mat[9] = rotn(2,1);	// New Y-axis
	mat[2] = rotn(0,2); mat[6] = rotn(1,2); mat[10] = rotn(2,2);	// New Z-axis
	// Apply pose transform
	glPushMatrix();
	glMultMatrixf(mat);
	
	// Draw a position point
	if (false) {
	glLineWidth(options_.rotation_axes_line_width);
	GLUquadricObj	*sphere_obj = gluNewQuadric();
	if (!sphere_obj) {LOG(ERROR)<<"gluNewQuadric for sphere not created"; return false;}
	glColor4f(1.,1.,0.,1.);
	gluQuadricDrawStyle(sphere_obj, GLU_FILL);	//GLU_LINE/GLU_FILL
	glPushMatrix();
	glScalef(options_.position_sphere_size*2.,
					 options_.position_sphere_size,
					 options_.position_sphere_size);
	gluSphere(sphere_obj, 1, 8, 8);
	gluDeleteQuadric(sphere_obj);
	glPopMatrix();
	}

	glDisable(GL_LIGHTING);  // Disable lights when drawing lines
	
	// Draw rotation axes
	glLineWidth(options_.rotation_axes_line_width);
	glPushMatrix();
	glScalef(options_.rotation_axes_line_length,
					 options_.rotation_axes_line_length,
					 options_.rotation_axes_line_length);
	// X axis
	glBegin(GL_LINES);
	glColor4f(1.,0.,0.,1.);
	glVertex3f(0.,0.,0.);
	glVertex3f(1.,0.,0.);
	glEnd();
	// Y axis
	glBegin(GL_LINES);
	glColor4f(0.,1.,0.,1.);
	glVertex3f(0.,0.,0.);
	glVertex3f(0.,1.,0.);
	glEnd();
	// Z axis
	glBegin(GL_LINES);
	glColor4f(0.,0.,1.,1.);
	glVertex3f(0.,0.,0.);
	glVertex3f(0.,0.,1.);
	glEnd();
	
	// Draw Target
	DrawTarget(options_.target_height, options_.target_width, options_.target_depth);
	
	glPopMatrix();
	
	// Draw position ellipsoid
	GLfloat emat[16];
	emat[3] = emat[7] = emat[11] = 0; emat[15] = 1; emat[12] = emat[13] = emat[14] = 0;
	const Eigen::Matrix3d& ev = pose.position_cov_eigenvectors_;
	const Eigen::Vector3d& el_sqrt = pose.position_cov_eigenvalues_sqrt_;
	emat[0] = ev(0,0); emat[1] = ev(1,0); emat[2] = ev(2,0);	// New X-axis
	emat[4] = ev(0,1); emat[5] = ev(1,1); emat[6] = ev(2,1);	// New Y-axis
	emat[8] = ev(0,2); emat[9] = ev(1,2); emat[10] = ev(2,2);	// New Z-axis	
	glLineWidth(options_.ellipsoid_line_width);
	glColor4f(1.,1.,0.,1.);
	glPushMatrix();
	glMultMatrixf(emat);
	glScalef(el_sqrt(0), el_sqrt(1), el_sqrt(2));
	DrawCircleXY();
	DrawCircleYZ();
	DrawCircleZX();
	glPopMatrix();
	
	// Draw rotation uncertainty disks
	glPushMatrix();
	glScalef(options_.rotation_axes_line_length,
					 options_.rotation_axes_line_length,
					 options_.rotation_axes_line_length);
	glLineWidth(options_.ellipsoid_line_width);
	glColor4f(1.,0.,0.,1.);
	glBegin(GL_LINE_LOOP);
	for (int i=0; i<16; i++) {
		glVertex3f(pose.rotation_cov_x_disk_points_(0,i),
							 pose.rotation_cov_x_disk_points_(1,i),
							 pose.rotation_cov_x_disk_points_(2,i));
	}
	glEnd();
	glColor4f(0.,1.,0.,1.);
	glBegin(GL_LINE_LOOP);
	for (int i=0; i<16; i++) {
		glVertex3f(pose.rotation_cov_y_disk_points_(0,i),
							 pose.rotation_cov_y_disk_points_(1,i),
							 pose.rotation_cov_y_disk_points_(2,i));
	}
	glEnd();
	glColor4f(0.,0.,1.,1.);
	glBegin(GL_LINE_LOOP);
	for (int i=0; i<16; i++) {
		glVertex3f(pose.rotation_cov_z_disk_points_(0,i),
							 pose.rotation_cov_z_disk_points_(1,i),
							 pose.rotation_cov_z_disk_points_(2,i));
	}
	glEnd();
	glPopMatrix();	

	// Remove pose transform
	glPopMatrix();	
	
	//glDisable(GL_BLEND);
	glEnable(GL_LIGHTING);
	
	return true;
}

bool Viewer::DrawVelocityState(const anantak::PoseStateDisplay& pose) {
	
	if (!pose.has_velocity_) return false;
	
	// Transform to the pose
	glPushMatrix();	// Pose rotation and position
	glMultMatrixd(pose.pose_T_.data());
	
	// Draw linear velocity
	glColor4f(1.,1.,1.,1.);
	glPushMatrix();	// velocity scale
	glScalef(options_.velocity_arrow_length_ratio,
					 options_.velocity_arrow_length_ratio,
					 options_.velocity_arrow_length_ratio);
	glPushMatrix();	// Draw arrow
	glMultMatrixd(pose.v_arrow_T_.data());
	//DrawArrow(pose.v_norm_, options_.velocity_arrow_radius_ratio, 12);
	DrawLine(pose.v_norm_);
	glPopMatrix();  // Draw arrow
	// Draw linear velocity uncertainty circles
	glPushMatrix();	// Draw circles
	glTranslated(pose.v_(0), pose.v_(1), pose.v_(2));
	glMultMatrixd(pose.v_circles_T_.data());
	glScalef(pose.v_cov_eigenvalues_sqrt_(0), pose.v_cov_eigenvalues_sqrt_(1), pose.v_cov_eigenvalues_sqrt_(2));
	glDisable(GL_LIGHTING);  // Disable lights when drawing lines
	DrawCircleXY();
	DrawCircleYZ();
	DrawCircleZX();	
	glEnable(GL_LIGHTING);  // Disable lights when drawing lines
	glPopMatrix();  // Draw circles
	glPopMatrix();  // velocity scale
	
	// Draw angular velocity
	glColor4f(1.,1.,0.,1.);
	glPushMatrix();	// velocity scale
	glScalef(options_.velocity_arrow_length_ratio,
					 options_.velocity_arrow_length_ratio,
					 options_.velocity_arrow_length_ratio);
	glPushMatrix();	// Draw arrow
	glMultMatrixd(pose.w_arrow_T_.data());
	//DrawArrow(pose.w_norm_, options_.velocity_arrow_radius_ratio, 12);
	DrawLine(pose.w_norm_);
	glPopMatrix();  // Draw arrow
	// Draw linear velocity uncertainty circles
	glPushMatrix();	// Draw circles
	glTranslated(pose.w_(0), pose.w_(1), pose.w_(2));
	glMultMatrixd(pose.w_circles_T_.data());
	glScalef(pose.w_cov_eigenvalues_sqrt_(0), pose.w_cov_eigenvalues_sqrt_(1), pose.w_cov_eigenvalues_sqrt_(2));
	glDisable(GL_LIGHTING);  // Disable lights when drawing lines
	DrawCircleXY();
	DrawCircleYZ();
	DrawCircleZX();	
	glEnable(GL_LIGHTING);  // Disable lights when drawing lines
	glPopMatrix();  // Draw circles
	glPopMatrix();  // velocity scale
	
	glPopMatrix(); 	// Pose rotation and position
	
	
	return true;
}

bool Viewer::DrawCamera(const anantak::CameraStateDisplay& camera) {
	
	glDisable(GL_LIGHTING);  // Disable lights when drawing lines
	glLineWidth(options_.rotation_axes_line_width);
	
	GLfloat mat[16];
	for (int i=0; i<16; i++) {mat[i] = float(camera.transform_(i));}
	// Apply pose transform
	glPushMatrix();
	glMultMatrixf(mat);
	
	// Draw rotation axes
	glScalef(options_.meters_per_pixel,
					 options_.meters_per_pixel,
					 options_.meters_per_pixel);
	// Camera
	glColor4f(1.,0.,1.,1.);
	glBegin(GL_LINE_LOOP);
	glVertex3f(camera.vertices_(0,1), camera.vertices_(1,1), camera.vertices_(2,1));
	glVertex3f(camera.vertices_(0,2), camera.vertices_(1,2), camera.vertices_(2,2));
	glVertex3f(camera.vertices_(0,3), camera.vertices_(1,3), camera.vertices_(2,3));
	glVertex3f(camera.vertices_(0,4), camera.vertices_(1,4), camera.vertices_(2,4));
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(camera.vertices_(0,0), camera.vertices_(1,0), camera.vertices_(2,0));
	glVertex3f(camera.vertices_(0,1), camera.vertices_(1,1), camera.vertices_(2,1));
	glVertex3f(camera.vertices_(0,0), camera.vertices_(1,0), camera.vertices_(2,0));
	glVertex3f(camera.vertices_(0,2), camera.vertices_(1,2), camera.vertices_(2,2));
	glVertex3f(camera.vertices_(0,0), camera.vertices_(1,0), camera.vertices_(2,0));
	glVertex3f(camera.vertices_(0,3), camera.vertices_(1,3), camera.vertices_(2,3));
	glVertex3f(camera.vertices_(0,0), camera.vertices_(1,0), camera.vertices_(2,0));
	glVertex3f(camera.vertices_(0,4), camera.vertices_(1,4), camera.vertices_(2,4));
	glVertex3f(camera.vertices_(0,0), camera.vertices_(1,0), camera.vertices_(2,0));
	glVertex3f(camera.vertices_(0,5), camera.vertices_(1,5), camera.vertices_(2,5));
	glEnd();
	
	glPopMatrix();
	glEnable(GL_LIGHTING);
	
}

QString Viewer::helpString() const {
  QString text("<h2>V i e w e r</h2>");
  text += "Use the mouse to move the camera around the object. ";
  text += "You can respectively revolve around, zoom and translate with the three mouse buttons. ";
  text += "Left and middle buttons pressed together rotate around the camera view direction axis<br><br>";
  text += "Pressing <b>Alt</b> and one of the function keys (<b>F1</b>..<b>F12</b>) defines a camera keyFrame. ";
  text += "Simply press the function key again to restore it. Several keyFrames define a ";
  text += "camera path. Paths are saved when you quit the application and restored at next start.<br><br>";
  text += "Press <b>F</b> to display the frame rate, <b>A</b> for the world axis, ";
  text += "<b>Alt+Return</b> for full screen mode and <b>Control+S</b> to save a snapshot. ";
  text += "See the <b>Keyboard</b> tab in this window for a complete shortcut list.<br><br>";
  text += "Double clicks automates single click actions: A left button double click aligns the closer axis with the camera (if close enough). ";
  text += "A middle button double click fits the zoom of the camera and the right button re-centers the scene.<br><br>";
  text += "A left button double click while holding right button pressed defines the camera <i>Revolve Around Point</i>. ";
  text += "See the <b>Mouse</b> tab and the documentation web pages for details.<br><br>";
  text += "Press <b>Escape</b> to exit the viewer.";
  return text;
}


} // namespace
