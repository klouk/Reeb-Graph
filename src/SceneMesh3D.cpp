#include "SceneMesh3D.h"
#include <queue>

#define SPLIT_INSTEAD_OF_INTERSECT 0
# define INF 0x3f3f3f3f
using namespace std;
using namespace vvr;
Mesh3DScene::Mesh3DScene()
{
    //! Load settings.
    vvr::Shape::DEF_LINE_WIDTH = 4;
    vvr::Shape::DEF_POINT_SIZE = 10;
    m_perspective_proj = true;
    m_bg_col = Colour("768E77");
    m_obj_col = Colour("454545");
    const string objDir = getBasePath() + "resources/obj/";
    const string objFile = objDir + "unicorn_low_low.obj";//flashlight//Phone_v02//unicorn_low_low//dragon_low_low//armadillo_low_low//bunny_low//cube/dolphin
    m_model_original = vvr::Mesh(objFile);
    reset();
}
void Mesh3DScene::mousePressed(int x, int y, int modif)
{

	 
}
void Mesh3DScene::reset()
{
    Scene::reset();

    //! Define plane
    m_plane_d = 0;
    m_plane = Plane(vec(0, 1, 1).Normalized(), m_plane_d);

    //! Define what will be vissible by default
    m_style_flag = 0;
    m_style_flag |= FLAG_SHOW_SOLID;
    m_style_flag |= FLAG_SHOW_WIRE;
    m_style_flag |= FLAG_SHOW_AXES;
    m_style_flag |= FLAG_SHOW_AABB;
	m_style_flag |= FLAG_SHOW_DUAL_GRAPH;
    m_style_flag |= FLAG_SHOW_PLANE;
	m_style_flag |= FLAG_SHOW_NEXT;
	 
}
bool first_pass = true;
void Mesh3DScene::resize()
{
    //! By Making `first_pass` static and initializing it to true,
    //! we make sure that the if block will be executed only once.

    // static bool first_pass = true;
	
    if (first_pass)
	{
		
        m_model_original.setBigSize(getSceneWidth() / 2);
        m_model_original.update();
        m_model = m_model_original;
		 


		cout << "epiloges gia na epileksete\n>>";
		cout << "1 gia dualgraph\n";
		cout << "2 gia GeodesicDistance\n";
		cout << "3 gia Convex3D\n";
		cout << "4 gia reeb_graph_height\n";
		cout << "5 gia reeb_graph_geodesic_distances\n";
		cout << "6 gia telika_akria\n";
		cout << "7 gia partition\n";
		cout << "8 gia simplification\n";
		cout << "9 gia motion\n";
		
		 

		cin >> ena;
		cout << "wait";
		 if (ena == 1) { dualgraph(); }
		 if (ena == 2) { GeodesicDistance(); }
		 if (ena == 3) { Convex3D(); }
		 if (ena == 4) { reeb_graph_height(); }
		 if (ena == 5) { reeb_graph_geodesic_distances(); }
		 if (ena == 6) { telika_akria(1); }
		 if (ena == 7) { partition(); }
		 if (ena == 8) { simplification(); }
		 if (ena == 9) { motion(); }

		 
		
		//dualgraph();

		//if (m_style_flag & FLAG_SHOW_GeodesicDistance)GeodesicDistance();
		//GeodesicDistance();
		//if (m_style_flag & FLAG_SHOW_Convex3D) Convex3D();
		//if (m_style_flag & FLAG_SHOW_reeb_graph_height)reeb_graph_height();
		//if (m_style_flag & FLAG_SHOW_telika_akria)telika_akria(1);
		//if (m_style_flag & FLAG_SHOW_reeb_graph_geodesic_distances)reeb_graph_geodesic_distances();
		//if (m_style_flag & FLAG_SHOW_partition)partition();
		//if (m_style_flag & FLAG_SHOW_simplification)simplification();
		//curvuture();
		
		//if (m_style_flag & FLAG_SHOW_GeodesicDistance) motion();
        first_pass = false;
    }
	 
}
void Mesh3DScene::arrowEvent(ArrowDir dir, int modif)
{
    math::vec n = m_plane.normal;
    if (dir == UP) m_plane_d += 1;
    if (dir == DOWN) m_plane_d -= 1;
    else if (dir == LEFT) n = math::float3x3::RotateY(DegToRad(1)).Transform(n);
    else if (dir == RIGHT) n = math::float3x3::RotateY(DegToRad(-1)).Transform(n);
    m_plane = Plane(n.Normalized(), m_plane_d);

    if (SPLIT_INSTEAD_OF_INTERSECT == false) {
        m_intersections.clear();
      
    }
    else {
        m_model = Mesh(m_model_original);
       
    }
}
void Mesh3DScene::keyEvent(unsigned char key, bool up, int modif)
{
    Scene::keyEvent(key, up, modif);
    key = tolower(key);

    switch (key)
    {
    case 's': m_style_flag ^= FLAG_SHOW_SOLID; break;
    case 'w': m_style_flag ^= FLAG_SHOW_WIRE; break;
    case 'n': m_style_flag ^= FLAG_SHOW_NORMALS; break;
    case 'a': m_style_flag ^= FLAG_SHOW_AXES; break;
    case 'p': m_style_flag ^= FLAG_SHOW_PLANE; break;
    case 'b': m_style_flag ^= FLAG_SHOW_AABB; break;
	case 'd': m_style_flag ^= FLAG_SHOW_DUAL_GRAPH; break;

	case 'x': m_style_flag ^= FLAG_SHOW_NEXT; {   }
	 










    }
}
void Mesh3DScene::draw()
{
	 
	if (m_style_flag & FLAG_SHOW_PLANE) {
		vvr::Colour colPlane(0x41, 0x14, 0xB3);
		float u = 20, v = 20;
		math::vec p0(plane_t.Point(-u, -v, math::vec(0, 0, 0)));
		math::vec p1(plane_t.Point(-u, v, math::vec(0, 0, 0)));
		math::vec p2(plane_t.Point(u, -v, math::vec(0, 0, 0)));
		math::vec p3(plane_t.Point(u, v, math::vec(0, 0, 0)));
		//math2vvr(math::Triangle(p0, p1, p2), colPlane).draw();
		//math2vvr(math::Triangle(p2, p1, p3), colPlane).draw();
	}




	if (m_style_flag & FLAG_SHOW_SOLID) m_model.draw(m_obj_col, SOLID);
	if (m_style_flag & FLAG_SHOW_WIRE) m_model.draw(Colour::black, WIRE);
	if (m_style_flag & FLAG_SHOW_NORMALS) m_model.draw(Colour::black, NORMALS);
	//if (m_style_flag & FLAG_SHOW_AXES) m_model.draw(Colour::black, AXES);

	//	simplified.draw(Colour::green, SOLID);
	//m_model_original.draw(Colour::green, SOLID);
	//	simplified.draw(Colour::black, WIRE);
	//	m_model_original.draw(Colour::black, WIRE);
	//	simplified.draw(Colour::black, WIRE);

	//redMesh.draw(Colour::black, WIRE);

	//////////////////////////DUAL_GRAPH/////////////////////////////////////////
	
	if (ena==1) {
		for (int i = 0; i < dual_graph.size(); i++) {

			dual_graph[i].draw();
		}
	}
	////////////////////////////////////////////////////////////////////////////
	//////////////////////////GEODESIAKES_APOSTASEIS/////////////////////////////////////////
	if (ena == 2) {
		 
			int metr = 0;
			for (const auto &t : tris)
			{
				float orio = (max_g - min_g) / 10;
				//echo(orio);
				if (geodesiakes[metr] < (orio * 1 + min_g)) { auto floor_tri = math2vvr(t, Colour::darkRed);     floor_tri.setSolidRender(true); floor_tri.draw(); }
				else if (geodesiakes[metr] < (orio * 2 + min_g)) { auto floor_tri = math2vvr(t, Colour::red);         floor_tri.setSolidRender(true); floor_tri.draw(); }
				else if (geodesiakes[metr] < (orio * 3 + min_g)) { auto floor_tri = math2vvr(t, Colour::darkOrange);  floor_tri.setSolidRender(true); floor_tri.draw(); }
				else if (geodesiakes[metr] < (orio * 4 + min_g)) { auto floor_tri = math2vvr(t, Colour::orange);      floor_tri.setSolidRender(true); floor_tri.draw(); }
				else if (geodesiakes[metr] < (orio * 5 + min_g)) { auto floor_tri = math2vvr(t, Colour::yellow);      floor_tri.setSolidRender(true); floor_tri.draw(); }
				else if (geodesiakes[metr] < (orio * 6 + min_g)) { auto floor_tri = math2vvr(t, Colour::yellowGreen); floor_tri.setSolidRender(true); floor_tri.draw(); }
				else if (geodesiakes[metr] < (orio * 7 + min_g)) { auto floor_tri = math2vvr(t, Colour::green);       floor_tri.setSolidRender(true); floor_tri.draw(); }
				else if (geodesiakes[metr] < (orio * 8 + min_g)) { auto floor_tri = math2vvr(t, Colour::darkGreen);   floor_tri.setSolidRender(true); floor_tri.draw(); }
				else if (geodesiakes[metr] < (orio * 9 + min_g)) { auto floor_tri = math2vvr(t, Colour::cyan);        floor_tri.setSolidRender(true); floor_tri.draw(); }
				else if (geodesiakes[metr] <= (orio * 10 + min_g)) { auto floor_tri = math2vvr(t, Colour::blue);        floor_tri.setSolidRender(true); floor_tri.draw(); }
				metr++;
			 
		}
	}
	////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////TELIKA_AKRAIA_SIMEIA////////////////////////////////////////
	if (ena == 6) {
		 
			//echo(telika_akraia_si.size());
			for (int g = 0; g < telika_akraia_si.size(); g++) {

				vec vecs = telika_akraia_si[g];
				Point3D temp1(vecs.x, vecs.y, vecs.z, Colour::red);
					temp1.draw();
			 
		}
	}
	////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////CONVEX_HULL_3D////////////////////////////////////////
	if (ena == 3) {
		for (int i = 0; i < trigwna_piramidas.size(); i++) {
			vvr::Colour colPlane(0x41, 0x14, 0xB3);
			//float u = 40, v = 40;

			if (trigwna_piramidas[i].x1 == 0 &&
				trigwna_piramidas[i].y1 == 0 &&
				trigwna_piramidas[i].z1 == 0 && trigwna_piramidas[i].x2 == 0 && trigwna_piramidas[i].y2 == 0 &&
				trigwna_piramidas[i].z2 == 0 && trigwna_piramidas[i].x3 == 0 &&
				trigwna_piramidas[i].y3 == 0 &&
				trigwna_piramidas[i].z3 == 0) {
			}
			else {
				vec a; a.x = trigwna_piramidas[i].x1; a.y = trigwna_piramidas[i].y1; a.z = trigwna_piramidas[i].z1;
				vec b; b.x = trigwna_piramidas[i].x2; b.y = trigwna_piramidas[i].y2; b.z = trigwna_piramidas[i].z2;
				vec c; c.x = trigwna_piramidas[i].x3; c.y = trigwna_piramidas[i].y3; c.z = trigwna_piramidas[i].z3;
				LineSeg3D line1 = LineSeg3D(a.x, a.y, a.z, b.x, b.y, b.z, Colour::magenta);
				line1.draw();
				LineSeg3D line2 = LineSeg3D(a.x, a.y, a.z, c.x, c.y, c.z, Colour::magenta);
				line2.draw();
				LineSeg3D line3 = LineSeg3D(c.x, c.y, c.z, b.x, b.y, b.z, Colour::magenta);
				line3.draw();
				a.x = trigwna_piramidas[i].x1 + 50; a.y = trigwna_piramidas[i].y1; a.z = trigwna_piramidas[i].z1;
				b.x = trigwna_piramidas[i].x2 + 50; b.y = trigwna_piramidas[i].y2; b.z = trigwna_piramidas[i].z2;
				c.x = trigwna_piramidas[i].x3 + 50; c.y = trigwna_piramidas[i].y3; c.z = trigwna_piramidas[i].z3;
				math2vvr(math::Triangle(a, b, c), colPlane).draw();
			}

		}
	}
	//piramida(piramida_points);

	////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////REEB_GRAPH_KATH'UPSOS-geodesiakes///////////////////////////////////////
	if (ena == 4) {
		greenMesh.draw(Colour::black, WIRE);
		for (int i = 0; i < reeb_lines.size(); i++) {

			 reeb_lines[i].draw();
		}
	}
	////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////REEB_GRAPH_GEODESIAKES///////////////////////////////////////
	if (ena == 5) {
		 
			for (int i = 0; i < reeb_lines.size(); i++) {

				reeb_lines[i].draw();
			 
		}
	}
	////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////MOTION///////////////////////////////////////////////////////////
	for (int i = 0; i < aksonas.size(); i++) {
		 //aksonas[i].draw();
	}
	for (int i = 0; i < mprosta_deksi.size(); i++) {
		//  mprosta_deksi[i].draw();// 

	}
	for (int i = 0; i < pisw_deksi.size(); i++) {
		// pisw_deksi[i].draw();// 

	}
	for (int i = 0; i < pisw_aristero.size(); i++) {
		//   pisw_aristero[i].draw();// 

	}
	for (int i = 0; i < mprosta_aristero.size(); i++) {
		//mprosta_aristero[i].draw();// 

	}
	for (int i = 0; i < oura.size(); i++) {
		 

		//oura[i].draw();
	}
	if (ena == 9) {
		if (true)moires = -moires;
		fliptale(peristrofis, moires);

		flipleg(mprosta_aristero_peris, moires, mprosta_aristero_poi);
		flipleg(mprosta_deksi_peris, -moires, mprosta_deksi_poi);
		flipleg(pisw_deksi_peris, -moires, pisw_deksi_poi);
		flipleg(pisw_aristero_peris, +moires, pisw_aristero_poi);
	}

	////////////////////////////////PARTITON////////////////////////////////////////////
	 if (ena == 7) {
		 for (int i = 0; i < trigwna_antikeimenwn.size(); i++) {
			 for (int j = 0; j < trigwna_antikeimenwn[i].size(); j++) {
				 Point3D temp1;
				 vec a = trigwna_antikeimenwn[i][j].v1();
				 vec b = trigwna_antikeimenwn[i][j].v2();
				 vec c = trigwna_antikeimenwn[i][j].v3();
				 math2vvr(math::Triangle(a, b, c), Colour(250 - 50 * i, 10 * i, 50 * i)).draw();

			 }
		 }
	 }
	////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////

 
	 
	vector<vec> &klouk_vertices = convexhull.getVertices();
	for (int g = 0; g<klouk_vertices.size(); g++) {

		vec vecs = klouk_vertices[g];
		Point3D temp1(vecs.x, vecs.y, vecs.z, Colour::green);
	//	temp1.draw();


	}

	/*
		for (int i=0;i<curvuture_vaues.size();i++)
		{
			float orio = (max_curv - min_curv) / 14;

			if (curvuture_vaues[i] < (orio * 1 + min_curv)) {Point3D temp1(m_model.getVertices()[i].x, m_model.getVertices()[i].y, m_model.getVertices()[i].z, Colour::darkRed); temp1.draw();}
			else if (curvuture_vaues[i] < (orio * 2 + min_curv)) { Point3D temp1(m_model.getVertices()[i].x, m_model.getVertices()[i].y, m_model.getVertices()[i].z, Colour::red); temp1.draw(); }
			else if (curvuture_vaues[i] < (orio * 3 + min_curv)) { Point3D temp1(m_model.getVertices()[i].x, m_model.getVertices()[i].y, m_model.getVertices()[i].z, Colour::darkOrange); temp1.draw(); }
			else if (curvuture_vaues[i] < (orio * 4 + min_curv)) { Point3D temp1(m_model.getVertices()[i].x, m_model.getVertices()[i].y, m_model.getVertices()[i].z, Colour::orange); temp1.draw(); }
			else if (curvuture_vaues[i] < (orio * 5 + min_curv)) { Point3D temp1(m_model.getVertices()[i].x, m_model.getVertices()[i].y, m_model.getVertices()[i].z, Colour::yellow); temp1.draw(); }
			else if (curvuture_vaues[i] < (orio * 6 + min_curv)) { Point3D temp1(m_model.getVertices()[i].x, m_model.getVertices()[i].y, m_model.getVertices()[i].z, Colour::yellowGreen); temp1.draw(); }
			else if (curvuture_vaues[i] < (orio * 7 + min_curv)) { Point3D temp1(m_model.getVertices()[i].x, m_model.getVertices()[i].y, m_model.getVertices()[i].z, Colour::green); temp1.draw(); }
			else if (curvuture_vaues[i] < (orio * 8 + min_curv)) { Point3D temp1(m_model.getVertices()[i].x, m_model.getVertices()[i].y, m_model.getVertices()[i].z, Colour::darkGreen); temp1.draw(); }
			else if (curvuture_vaues[i] < (orio * 9 + min_curv)) { Point3D temp1(m_model.getVertices()[i].x, m_model.getVertices()[i].y, m_model.getVertices()[i].z, Colour::cyan); temp1.draw(); }
			else if (curvuture_vaues[i] <= (orio * 10 + min_curv)) { Point3D temp1(m_model.getVertices()[i].x, m_model.getVertices()[i].y, m_model.getVertices()[i].z, Colour::blue); temp1.draw(); }
			else if (curvuture_vaues[i] <= (orio * 11 + min_curv)) { Point3D temp1(m_model.getVertices()[i].x, m_model.getVertices()[i].y, m_model.getVertices()[i].z, Colour::magenta); temp1.draw(); }
			else if (curvuture_vaues[i] <= (orio * 12 + min_curv)) { Point3D temp1(m_model.getVertices()[i].x, m_model.getVertices()[i].y, m_model.getVertices()[i].z, Colour::grey); temp1.draw(); }
			else if (curvuture_vaues[i] <= (orio * 13 + min_curv)) { Point3D temp1(m_model.getVertices()[i].x, m_model.getVertices()[i].y, m_model.getVertices()[i].z, Colour::black); temp1.draw(); }
			else if (curvuture_vaues[i] <= (orio * 14 + min_curv)) { Point3D temp1(m_model.getVertices()[i].x, m_model.getVertices()[i].y, m_model.getVertices()[i].z, Colour::white); temp1.draw(); }
			 
		} 
		*/
	
	 
 
	 

		for (int i = 0; i < antikeimena_mesh.size(); i++) {
			//antikeimena_mesh[i].draw(Colour(250 - 50 * i, 10 * i, 50 * i), SOLID);
		}
		for (int i = 0; i < antikeimena_mesh.size(); i++) {
				//antikeimena_mesh[i].draw(Colour(250 - 50 * i, 10 * i, 50 * i), SOLID);
		}
		for (int i = 0; i < intex_antikeinwn.size(); i++) {
			for (int j = 0; j < intex_antikeinwn[i].size(); j++) {
				//antikeimena_mesh[intex_antikeinwn[i][j]].draw(Colour(250 - 50 * i, 10 * i, 50 * i), SOLID);
				//antikeimena_mesh[intex_antikeinwn[i][j]].draw(Colour(250 - 50 * i, 10 * i, 50 * i), SOLID);

			}
		}
 

	 for (int i = 0; i < meloi.size(); i++) {
		 for (int j = 0; j < meloi[i].size(); j++) {	 
			//  meloi[i][j].draw();// 


			  vec a;
			  a.x = meloi[i][j].x1;
			  a.y = meloi[i][j].y1;
			  a.z = meloi[i][j].z1;
			//  Point3D temp1(a.x, a.y, a.z, Colour(250 - 50 * i, 10 * i, 50 * i));
			//  temp1.draw();
			  vec b;
			  b.x = meloi[i][j].x2;
			  b.y = meloi[i][j].y2;
			  b.z = meloi[i][j].z2;
			   //  temp1= Point3D(b.x, b.y, b.z, Colour(250 - 50 * i, 10 * i, 50 * i));
			 // temp1.draw();



			    // math2vvr(math::Triangle(a, b, c), Colour(250 - 50 * i, 10 * i, 50 * i)).draw();
		 }
	 }

}
int main(int argc, char* argv[])
{
    try {
        return vvr::mainLoop(argc, argv, new Mesh3DScene);
    }
    catch (std::string exc) {
        cerr << exc << endl;
        return 1;
    }
    catch (...)
    {
        cerr << "Unknown exception" << endl;
        return 1;
    }
}
void Mesh3DScene::GeodesicDistance() {
	
	 dualgraph();

	//echo(kentra_dual_graph.size());
	float max, min;
	max = 0;
	min = 228360000;
	for (int i = 0; i < kentra_dual_graph.size(); i++) {
		 
		Dijkstra(i, max, min);
		//echo(i);
	 
	}
	max_g = 0;
	min_g = inf;
	for (int i = 0; i < kentra_dual_graph.size(); i++) {
		vvr::Triangle &temp =m_model.getTriangles()[i];
		geodesiakes[i] = abs(geodesiakes[i] - min) / max;
		//edw briskw maxg kai ming gia na kanw meta stin draw tin katigoropoisi
		if (max_g < geodesiakes[i]) { max_g = geodesiakes[i]; }
		if (min_g > geodesiakes[i]) { min_g = geodesiakes[i]; }
		///e//cho(geodesiakes[i]);

		tris.push_back(math::Triangle(temp.v1(), temp.v2(), temp.v3()));
	}

}
void Mesh3DScene::Poligwnopoisi() {

	vvr::Mesh  mesh = m_model_original;
	vector<vvr::Triangle> &klouk_triangles = mesh.getTriangles();
	vector<vec> &klouk_vertices = mesh.getVertices();

	float temp_em = embadon(Triangle3D(klouk_triangles[0].v1().x, klouk_triangles[0].v1().y, klouk_triangles[0].v1().z, klouk_triangles[0].v2().x, klouk_triangles[0].v2().y, klouk_triangles[0].v2().z, klouk_triangles[0].v3().x, klouk_triangles[0].v3().y, klouk_triangles[0].v3().z));
	temp_em= temp_em + embadon(Triangle3D(klouk_triangles[1].v1().x, klouk_triangles[1].v1().y, klouk_triangles[1].v1().z, klouk_triangles[1].v2().x, klouk_triangles[1].v2().y, klouk_triangles[1].v2().z, klouk_triangles[1].v3().x, klouk_triangles[1].v3().y, klouk_triangles[1].v3().z));


	for (int i = 0; i < klouk_vertices.size(); i++) {
		vector<int> intex;
		vec temp = klouk_vertices[0];
		for (int j = 0; j < klouk_triangles.size(); j++) {
			vec a = klouk_triangles[0].v1();
			vec b = klouk_triangles[0].v2();
			vec c = klouk_triangles[0].v3();
			if ((a.x == temp.x && a.y == temp.y && a.z == temp.z) || (b.x == temp.x && b.y == temp.y && b.z == temp.z) || (c.x == temp.x && c.y == temp.y && c.z == temp.z)) {
				intex.push_back(j);
			
			}
		
			
		
		
		}
	
	
	
	
	
	}



















}
void Mesh3DScene::dualgraph() {
	 
	vvr::Mesh  mesh = m_model_original;
	vector<vec>  graph3;
	graph.resize(mesh.getTriangles().size(),vector<float>(mesh.getTriangles().size()));
	kanonika.resize(mesh.getTriangles().size()+1, 0);
	for (int i = 0; i < mesh.getTriangles().size(); ++i) {
		for (int j = 0; j < mesh.getTriangles().size(); ++j) {
			graph[i][j]=0;
		}
	}
	int metritis=0;
	for (int i = 0; i < mesh.getTriangles().size(); ++i) {//
		vvr::Triangle &temp = mesh.getTriangles()[i];

		const vec kentro = temp.getCenter();
		vec tempkento;
		tempkento.x = kentro.x;
		tempkento.y = kentro.y;
		tempkento.z = kentro.z;
		kentra_dual_graph.push_back(tempkento);
		vector<int> a;

		FindAdjacentTriangles(i, a);

		for (int k = 0; k < a.size(); k++) {
			vvr::Triangle &tempk = mesh.getTriangles()[a[k]];
			const vec kentro2 = tempk.getCenter();
			vec tempkento2;
			tempkento2.x = kentro2.x;
			tempkento2.y = kentro2.y;
			tempkento2.z = kentro2.z;
			vvr::LineSeg3D line;
			line = LineSeg3D(tempkento.x, tempkento.y, tempkento.z, tempkento2.x, tempkento2.y, tempkento2.z, vvr::Colour::blue);
			int metr = 0;
			/*
			   for (int j = 0; j < dual_graph.size(); j++) {
				   if (dual_graph[j].x1==line.x1 && dual_graph[j].x2 == line.x2 && dual_graph[j].y1 == line.y1 && dual_graph[j].y2 == line.y2 && dual_graph[j].z1 == line.z1 && dual_graph[j].z2 == line.z2) {
					   metr = 1;
				   }
				   if (dual_graph[j].x1 == line.x2 && dual_graph[j].x2 == line.x1 && dual_graph[j].y1 == line.y2 && dual_graph[j].y2 == line.y1 && dual_graph[j].z1 == line.z2 && dual_graph[j].z2 == line.z1) {
					   metr = 1;
				   }
			   }
				*/
			graphfinal2.resize(graphfinal2.size() + 1, vector<float>(3));
			graphfinal2[metritis][0] = float(i);
			graphfinal2[metritis][1] = float(a[k]);
			graphfinal2[metritis][2] = tempkento2.Distance(tempkento);
			metritis++;

			if (metr == 0) {
				dual_graph.push_back(line);
			}
		}
		if (a.size() != 3) {
		//	kanonika[i] = 1;

		}
		for (int t = 0; t < 3 - a.size(); t++) {
			//graphfinal2.resize(graphfinal2.size() + 1, vector<float>(3));
			//graphfinal2[metritis][0] = float(i);
			//graphfinal2[metritis][1] = float(i);
			//graphfinal2[metritis][2] = 228360.0;
			//metritis++;
		}
		if (i != 0) kanonika[i] = kanonika[i - 1] + a.size();
	
	 
		if(i== mesh.getTriangles().size()-1) kanonika[i+1]= kanonika[i] + a.size();
	}
 
	
	 
}
void Mesh3DScene::FindAdjacentTriangles(int i, vector<int> &a) {

	vvr::Mesh  mesh = m_model;
	vvr::Triangle temp1 = mesh.getTriangles()[i];
	int metr1 = 0, metr2 = 0;
	for (int j = 0; j < mesh.getTriangles().size(); ++j) {
		vvr::Triangle temp2 = mesh.getTriangles()[j];
		
		if (i != j) {
			if ((temp1.v1().x == temp2.v1().x && temp1.v1().y == temp2.v1().y && temp1.v1().z == temp2.v1().z) || (temp1.v1().x == temp2.v2().x && temp1.v1().y == temp2.v2().y && temp1.v1().z == temp2.v2().z) || (temp1.v1().x == temp2.v3().x && temp1.v1().y == temp2.v3().y && temp1.v1().z == temp2.v3().z)) { metr1++; }
			if ((temp1.v2().x == temp2.v1().x && temp1.v2().y == temp2.v1().y && temp1.v2().z == temp2.v1().z) || (temp1.v2().x == temp2.v2().x && temp1.v2().y == temp2.v2().y && temp1.v2().z == temp2.v2().z) || (temp1.v2().x == temp2.v3().x && temp1.v2().y == temp2.v3().y && temp1.v2().z == temp2.v3().z)) { metr1++; }
			if ((temp1.v3().x == temp2.v1().x && temp1.v3().y == temp2.v1().y && temp1.v3().z == temp2.v1().z) || (temp1.v3().x == temp2.v2().x && temp1.v3().y == temp2.v2().y && temp1.v3().z == temp2.v2().z) || (temp1.v3().x == temp2.v3().x && temp1.v3().y == temp2.v3().y && temp1.v3().z == temp2.v3().z)) { metr1++; }
			if (metr1 == 2) {
				//a[metr2] = j;
				a.push_back(j);
				metr2++;
			}metr1 = 0;

			if (metr2 == 3) { return; }
		}
	}
	return;


}
typedef pair<float, int> iPair;
struct compare
{
	bool operator()(iPair& l, iPair& r)
	{	
		return l.first > r.first;
	}
};
void Mesh3DScene::simplification() {
	//echo(m_model.getTriangles().size());
	vector<vector<vector<float>>> qerror;
	qerror.resize(m_model.getTriangles().size());
	for (int i = 0; i < m_model.getTriangles().size(); i++) {
		qerror[i].resize(4);
	}
	for (int i = 0; i < m_model.getTriangles().size(); i++) {
		for (int j = 0; j < 4; j++) {
			qerror[i][j].resize(4);
		}
	}
	////gia kathe trigwno briskw to palne equtation kai sximatizw ton pinaka
	for (int i = 0; i < m_model.getTriangles().size(); i++) {
		vec ap = m_model.getTriangles()[i].v1();
		vec bp = m_model.getTriangles()[i].v2();
		vec cp = m_model.getTriangles()[i].v3();
		float a = (bp.y - ap.y)*(cp.z - ap.z) - (cp.y - ap.y)*(bp.z - ap.z);
		float b = (bp.z - ap.z)*(cp.x - ap.x) - (cp.z - ap.z)*(bp.x - ap.x);
		float c = (bp.x - ap.x)*(cp.y - ap.y) - (cp.x - ap.x)*(bp.y - ap.y);
		float d = -(a*ap.x + b*ap.y + c*ap.z);

		qerror[i][0][0] = a*a;
		qerror[i][0][1] = a*b;
		qerror[i][0][2] = a*c;
		qerror[i][0][3] = a*d;

		qerror[i][1][0] = a*b;
		qerror[i][1][1] = b*b;
		qerror[i][1][2] = b*c;
		qerror[i][1][3] = b*d;

		qerror[i][2][0] = a*c;
		qerror[i][2][1] = b*c;
		qerror[i][2][2] = c*c;
		qerror[i][2][3] = c*d;

		qerror[i][3][0] = a*d;
		qerror[i][3][1] = d*b;
		qerror[i][3][2] = d*c;
		qerror[i][3][3] = d*d;

	}

	vector<vector<vector<float>>> qerror_vector;
	qerror_vector.resize(m_model.getVertices().size());
	for (int i = 0; i < m_model.getVertices().size(); i++) {
		qerror_vector[i].resize(4);
	}
	for (int i = 0; i < m_model.getVertices().size(); i++) {
		for (int j = 0; j < 4; j++) {
			qerror_vector[i][j].resize(4, 0);


		}
	}
	///////gia ka8e kombo sto mesh pernw to a8roisma tvn pinakwn twn trigwnwn pou anikei
	for (int i = 0; i < m_model.getVertices().size(); i++) {
		for (int j = 0; j < m_model.getTriangles().size(); j++) {
			if (i == m_model.getTriangles()[j].v[0] || i == m_model.getTriangles()[j].v[1] || i == m_model.getTriangles()[j].v[2]) {

				qerror_vector[i][0][0] = qerror_vector[i][0][0] + qerror[j][0][0];
				qerror_vector[i][0][1] = qerror_vector[i][0][1] + qerror[j][0][1];
				qerror_vector[i][0][2] = qerror_vector[i][0][2] + qerror[j][0][2];
				qerror_vector[i][0][3] = qerror_vector[i][0][3] + qerror[j][0][3];

				qerror_vector[i][1][0] = qerror_vector[i][1][0] + qerror[j][1][0];
				qerror_vector[i][1][1] = qerror_vector[i][1][1] + qerror[j][1][1];
				qerror_vector[i][1][2] = qerror_vector[i][1][2] + qerror[j][1][2];;
				qerror_vector[i][1][3] = qerror_vector[i][1][3] + qerror[i][1][3];

				qerror_vector[i][2][0] = qerror_vector[i][2][0] + qerror[j][2][0];
				qerror_vector[i][2][1] = qerror_vector[i][2][1] + qerror[j][2][1];
				qerror_vector[i][2][2] = qerror_vector[i][2][2] + qerror[j][2][2];
				qerror_vector[i][2][3] = qerror_vector[i][2][3] + qerror[j][2][3];

				qerror_vector[i][3][0] = qerror_vector[i][3][0] + qerror[j][3][0];
				qerror_vector[i][3][1] = qerror_vector[i][3][1] + qerror[j][3][1];
				qerror_vector[i][3][2] = qerror_vector[i][3][2] + qerror[j][3][2];
				qerror_vector[i][3][3] = qerror_vector[i][3][3] + qerror[j][3][3];
			}
		}
	}

	vector<vector<vector<float>>> qerror_vector_edges;
	qerror_vector_edges.resize((m_model.getTriangles().size() * 3)/2);
	for (int i = 0; i < (m_model.getTriangles().size() * 3)/2; i++) {

		qerror_vector_edges[i].resize(4);
	}


	for (int i = 0; i < (m_model.getTriangles().size() * 3)/2; i++) {
		for (int j = 0; j < 4; j++) {
			qerror_vector_edges[i][j].resize(4, 0);


		}
	}
	vector<pair<int, int>> proced;

	vector<float> lathoi;

	int metritis = 0;



	for (int i = 0; i < m_model.getVertices().size(); i++) {
		 
		vec edge1 = m_model.getVertices()[i];
				 
		      float aa, bb, cc;
					aa = edge1.x;
					bb = edge1.y;
					cc = edge1.z;

				float ee, ff, gg, hh, kk, ll, mm, nn, oo, pp, qq, rr, ss, tt, xx, ww;
				 
				 ee = qerror_vector[i][0][0];
				 ff = qerror_vector[i][0][1];
				 gg = qerror_vector[i][0][2];
				 hh = qerror_vector[i][0][3];

				 kk = qerror_vector[i][1][0];
				 ll = qerror_vector[i][1][1];
				 mm = qerror_vector[i][1][2];
				 nn = qerror_vector[i][1][3];
				 oo = qerror_vector[i][2][0];
				 pp = qerror_vector[i][2][1];
				 qq = qerror_vector[i][2][2];
				 rr = qerror_vector[i][2][3];
				 ss = qerror_vector[i][3][0];
				 tt = qerror_vector[i][3][1];
				 xx = qerror_vector[i][3][2];
				 ww = qerror_vector[i][3][3];
				lathoi.push_back(abs(ww + aa*(ss + aa*ee + bb*kk + cc*oo) + bb*(tt + aa*ff + bb*ll + cc*pp) + cc*(xx + aa*gg + bb*mm + cc*qq) + aa*hh + bb*nn + cc*rr));
	}
	/*
	for (int i = 0; i < m_model.getTriangles().size(); i++) {
		vvr::Triangle tris = m_model.getTriangles()[i];
		for (int j = 0; j < 3; j++) {
			bool flag = true;
			for (int met = 0; met < proced.size(); met++) {
				if (j == 0){if ((proced[met].first == tris.v[0] && proced[met].second == tris.v[1]) || (proced[met].first == tris.v[1] && proced[met].second == tris.v[0])) { flag = false; break; }}
				if (j == 1){if ((proced[met].first == tris.v[0] && proced[met].second == tris.v[2]) || (proced[met].first == tris.v[2] && proced[met].second == tris.v[0])) { flag = false; break; }}
				if (j == 2){if ((proced[met].first == tris.v[1] && proced[met].second == tris.v[2]) || (proced[met].first == tris.v[2] && proced[met].second == tris.v[1])) { flag = false; break; }}


			}
			if (flag == true) {
				vec edge1;
				int k, l;
				float aa, bb, cc;
				if (j == 0) {
					//edge1.x = (tris.v1().x + tris.v2().x) / 2;
					//edge1.y = (tris.v1().y + tris.v2().y) / 2;
					//edge1.z = (tris.v1().z + tris.v2().z) / 2;

					//aa = edge1.x;
					//bb = edge1.y;
					//cc = edge1.z;

					aa = tris.v1().x;
					bb = tris.v1().y;
					cc = tris.v1().z;

					k = 0;
					l = 1;
				}
				else if (j == 1) {

				//	edge1.x = (tris.v1().x + tris.v3().x) / 2;
				//	edge1.y = (tris.v1().y + tris.v3().y) / 2;
					//edge1.z = (tris.v1().z + tris.v3().z) / 2;
				//	aa = edge1.x;
				//	bb = edge1.y;
					//cc = edge1.z;
					aa = tris.v2().x;
					bb = tris.v2().y;
					cc = tris.v2().z;

					k = 0;
					l = 2;



				}
				else if (j == 2) {

					//edge1.x = (tris.v3().x + tris.v2().x) / 2;
					//edge1.y = (tris.v3().y + tris.v2().y) / 2;
					//edge1.z = (tris.v3().z + tris.v2().z) / 2;
					//aa = edge1.x;
					//bb = edge1.y;
					//cc = edge1.z;
					aa = tris.v3().x;
					bb = tris.v3().y;
					cc = tris.v3().z;
					k = 1;
					l = 2;

				}
				float ee, ff, gg, hh, kk, ll, mm, nn, oo, pp, qq, rr, ss, tt, xx, ww;
				proced.push_back(make_pair(tris.v[k], tris.v[l]));


				qerror_vector_edges[metritis][0][0] = qerror_vector[tris.v[k]][0][0] + qerror[tris.v[l]][0][0]; ee = qerror_vector_edges[metritis][0][0];
				qerror_vector_edges[metritis][0][1] = qerror_vector[tris.v[k]][0][1] + qerror[tris.v[l]][0][1]; ff = qerror_vector_edges[metritis][0][1];
				qerror_vector_edges[metritis][0][2] = qerror_vector[tris.v[k]][0][2] + qerror[tris.v[l]][0][2]; gg = qerror_vector_edges[metritis][0][2];
				qerror_vector_edges[metritis][0][3] = qerror_vector[tris.v[k]][0][3] + qerror[tris.v[l]][0][3]; hh = qerror_vector_edges[metritis][0][3];

				qerror_vector_edges[metritis][1][0] = qerror_vector[tris.v[k]][1][0] + qerror[tris.v[l]][1][0]; kk = qerror_vector_edges[metritis][1][0];
				qerror_vector_edges[metritis][1][1] = qerror_vector[tris.v[k]][1][1] + qerror[tris.v[l]][1][1]; ll = qerror_vector_edges[metritis][1][1];
				qerror_vector_edges[metritis][1][2] = qerror_vector[tris.v[k]][1][2] + qerror[tris.v[l]][1][2]; mm = qerror_vector_edges[metritis][1][2];
				qerror_vector_edges[metritis][1][3] = qerror_vector[tris.v[k]][1][3] + qerror[tris.v[l]][1][3]; nn = qerror_vector_edges[metritis][1][3];

				qerror_vector_edges[metritis][2][0] = qerror_vector[tris.v[k]][2][0] + qerror[tris.v[l]][2][0]; oo = qerror_vector_edges[metritis][2][0];
				qerror_vector_edges[metritis][2][1] = qerror_vector[tris.v[k]][2][1] + qerror[tris.v[l]][2][1]; pp = qerror_vector_edges[metritis][2][1];
				qerror_vector_edges[metritis][2][2] = qerror_vector[tris.v[k]][2][2] + qerror[tris.v[l]][2][2]; qq = qerror_vector_edges[metritis][2][2];
				qerror_vector_edges[metritis][2][3] = qerror_vector[tris.v[k]][2][3] + qerror[tris.v[l]][2][3]; rr = qerror_vector_edges[metritis][2][3];

				qerror_vector_edges[metritis][3][0] = qerror_vector[tris.v[k]][3][0] + qerror[tris.v[l]][3][0]; ss = qerror_vector_edges[metritis][3][0];
				qerror_vector_edges[metritis][3][1] = qerror_vector[tris.v[k]][3][1] + qerror[tris.v[l]][3][1]; tt = qerror_vector_edges[metritis][3][1];
				qerror_vector_edges[metritis][3][2] = qerror_vector[tris.v[k]][3][2] + qerror[tris.v[l]][3][2]; xx = qerror_vector_edges[metritis][3][2];
				qerror_vector_edges[metritis][3][3] = qerror_vector[tris.v[k]][3][3] + qerror[tris.v[l]][3][3]; ww = qerror_vector_edges[metritis][3][3];

				lathoi.push_back(abs(ww + aa*(ss + aa*ee + bb*kk + cc*oo) + bb*(tt + aa*ff + bb*ll + cc*pp) + cc*(xx + aa*gg + bb*mm + cc*qq) + aa*hh + bb*nn + cc*rr));
				metritis++;


			}
		}



	}




	*/
	int h = 0;
	h = 1;

	float min=inf;
	float max=0;
	for (int i = 0; i < lathoi.size(); i++) {
		if ( lathoi[i] > max)max =  lathoi[i];
		if (lathoi[i] < min)min = lathoi[i];
	}
	float bima = (max - min) / 50;
	int size_model= m_model.getTriangles().size();
	vector<int> procceced;
	procceced.resize(lathoi.size(), 0);

	for (int j = 0; j < m_model.getTriangles().size(); j++) {
		vvr::Triangle tris = m_model.getTriangles()[j];
		if ((lathoi[tris.vi1] + lathoi[tris.vi2] < min + 25*bima) && procceced[tris.vi1] != 1 && procceced[tris.vi2] != 1) {

			procceced[tris.vi1] = 1;
			procceced[tris.vi2] = 1;

			vec temp;
			temp.x = (m_model.getVertices()[tris.vi1].x + m_model.getVertices()[tris.vi2].x) / 2;
			temp.y = (m_model.getVertices()[tris.vi1].y + m_model.getVertices()[tris.vi2].y) / 2;
			temp.z = (m_model.getVertices()[tris.vi1].z + m_model.getVertices()[tris.vi2].z) / 2;
		
			m_model.getVertices()[tris.vi1].x = temp.x;
			m_model.getVertices()[tris.vi1].y = temp.y;
			m_model.getVertices()[tris.vi1].z = temp.z;

			m_model.getVertices()[tris.vi2].x = temp.x;
			m_model.getVertices()[tris.vi2].y = temp.y;
			m_model.getVertices()[tris.vi2].z = temp.z;
		
		
		
		}
		else if ((lathoi[tris.vi3] + lathoi[tris.vi2] < min + bima) && procceced[tris.vi3] != 1 && procceced[tris.vi2] != 1) {
	
			procceced[tris.vi3] = 1;
			procceced[tris.vi2] = 1;

			vec temp;
			temp.x = (m_model.getVertices()[tris.vi3].x + m_model.getVertices()[tris.vi2].x) / 2;
			temp.y = (m_model.getVertices()[tris.vi3].y + m_model.getVertices()[tris.vi2].y) / 2;
			temp.z = (m_model.getVertices()[tris.vi3].z + m_model.getVertices()[tris.vi2].z) / 2;

			m_model.getVertices()[tris.vi3].x = temp.x;
			m_model.getVertices()[tris.vi3].y = temp.y;
			m_model.getVertices()[tris.vi3].z = temp.z;

			m_model.getVertices()[tris.vi2].x = temp.x;
			m_model.getVertices()[tris.vi2].y = temp.y;
			m_model.getVertices()[tris.vi2].z = temp.z;



		}
		else if ((lathoi[tris.vi3] + lathoi[tris.vi1] < min + bima) && procceced[tris.vi3] != 1 && procceced[tris.vi1] != 1) {

			procceced[tris.vi3] = 1;
			procceced[tris.vi1] = 1;

			vec temp;
			temp.x = (m_model.getVertices()[tris.vi3].x + m_model.getVertices()[tris.vi1].x) / 2;
			temp.y = (m_model.getVertices()[tris.vi3].y + m_model.getVertices()[tris.vi1].y) / 2;
			temp.z = (m_model.getVertices()[tris.vi3].z + m_model.getVertices()[tris.vi1].z) / 2;

			m_model.getVertices()[tris.vi3].x = temp.x;
			m_model.getVertices()[tris.vi3].y = temp.y;
			m_model.getVertices()[tris.vi3].z = temp.z;

			m_model.getVertices()[tris.vi1].x = temp.x;
			m_model.getVertices()[tris.vi1].y = temp.y;
			m_model.getVertices()[tris.vi1].z = temp.z;



		}

	
	
	}

	/*
	for (int i = 0; i < -1;i++){/// lathoi.size(); i++) {
		//if((m_model.getVertices()[proced[i].second].x != inf && m_model.getVertices()[proced[i].second].y != inf && m_model.getVertices()[proced[i].second].z != inf) || (m_model.getVertices()[proced[i].first].x != inf && m_model.getVertices()[proced[i].first].y != inf && m_model.getVertices()[proced[i].first].z != inf)){
		if (abs(lathoi[i]) <min+bima*5 )
		{
			//int index;
			//for (int j = 0; j < m_model.getTriangles().size(); j++) {
				//vvr::Triangle tris = m_model.getTriangles()[j];

				//if ((tris.v[0] == proced[i].second && tris.v[1] == proced[i].first) || (tris.v[1] == proced[i].second && tris.v[0] == proced[i].first)) {
				
					vec temp;
					temp.x= (m_model.getVertices()[proced[i].first].x + m_model.getVertices()[proced[i].second].x) / 2;
					temp.y= (m_model.getVertices()[proced[i].first].y + m_model.getVertices()[proced[i].second].y) / 2;
					temp.z= (m_model.getVertices()[proced[i].first].z + m_model.getVertices()[proced[i].second].z) / 2;
					pointset3.push_back(m_model.getVertices()[proced[i].second]);
					pointset.push_back(m_model.getVertices()[proced[i].first]);

					akraia_simeia.push_back(temp);
					
				 	m_model.getVertices()[proced[i].first].x = temp.x;
					m_model.getVertices()[proced[i].first].y = temp.y;
				 	m_model.getVertices()[proced[i].first].z = temp.z;
					 
				 	m_model.getVertices()[proced[i].second].x = temp.x;
				 	m_model.getVertices()[proced[i].second].y = temp.y;
				 	m_model.getVertices()[proced[i].second].z = temp.z;

					
					int deik1 = proced[i].first;
					int deik2 = proced[i].second;

					for (int j = 0; j < m_model.getTriangles().size(); j++) {
						vvr::Triangle tris = m_model.getTriangles()[j];
						if (((tris.v[0] == deik1 && tris.v[1] == deik2) || (tris.v[1] == deik2 && tris.v[0] == deik1)) || ((tris.v[0] == deik1 && tris.v[2] == deik2) || (tris.v[2] == deik2 && tris.v[0] == deik1)) || ((tris.v[2] == deik1 && tris.v[1] == deik2) || (tris.v[1] == deik2 && tris.v[2] == deik1))) {
							m_model.getTriangles().erase(m_model.getTriangles().begin() + j);
							j--;
						}
					
					
					
					}


					for (int y = 0; y < proced.size(); y++) {
					
						if (deik1 == proced[y].first || deik1 == proced[y].second || deik2 == proced[y].first || deik2 == proced[y].second) {
						
							proced.erase(proced.begin() + y);
							lathoi.erase(lathoi.begin() + y);
						}
					
					
					
					}
					//aksonas.push_back(LineSeg3D(m_model.getVertices()[proced[i].first].x, m_model.getVertices()[proced[i].first].y, m_model.getVertices()[proced[i].first].z, m_model.getVertices()[proced[i].second].x, m_model.getVertices()[proced[i].second].y, m_model.getVertices()[proced[i].second].z));

				//
				
					//m_model.getVertices()[proced[i].second].x =  m_model.getVertices()[proced[i].first].x;
					// 	m_model.getVertices()[proced[i].second].y = m_model.getVertices()[proced[i].first].y;
					// m_model.getVertices()[proced[i].second].z = m_model.getVertices()[proced[i].first].z;
				
				//}



				/*
				if (tris.v[0] == proced[i].second) {
					int intex1 = tris.v[1];
					int intex2 = tris.v[2];
					m_model.getTriangles().erase(m_model.getTriangles().begin() + j);
					j--;
					m_model.getVertices()[proced[i].first].x = (m_model.getVertices()[proced[i].first].x + m_model.getVertices()[proced[i].second].x) / 2;
					m_model.getVertices()[proced[i].first].y = (m_model.getVertices()[proced[i].first].y + m_model.getVertices()[proced[i].second].y) / 2;
					m_model.getVertices()[proced[i].first].z = (m_model.getVertices()[proced[i].first].z + m_model.getVertices()[proced[i].second].z) / 2;
					

					 m_model.getTriangles().push_back(vvr::Triangle(&m_model.getVertices(), proced[i].first, intex1, intex2));


					//////////////////
					

					redMesh.getVertices().push_back(tris.v1());
					redMesh.getVertices().push_back(tris.v2());
					redMesh.getVertices().push_back(m_model.getVertices()[proced[i].first]);
					index = redMesh.getVertices().size() - 3;
					redMesh.getTriangles().push_back(vvr::Triangle(&redMesh.getVertices(), index, index + 1, index + 2));
					//////////////////////

				}
				else if (tris.v[1] == proced[i].second)
				{
					int intex1 = tris.v[0];
					int intex2 = tris.v[2];
					m_model.getTriangles().erase(m_model.getTriangles().begin() + j);
					j--;

					

					 m_model.getTriangles().push_back(vvr::Triangle(&m_model.getVertices(), proced[i].first, intex1, intex2));
					//////////////////
					m_model.getVertices()[proced[i].first].x = (m_model.getVertices()[proced[i].first].x + m_model.getVertices()[proced[i].second].x) / 2;
					m_model.getVertices()[proced[i].first].y = (m_model.getVertices()[proced[i].first].y + m_model.getVertices()[proced[i].second].y) / 2;
					m_model.getVertices()[proced[i].first].z = (m_model.getVertices()[proced[i].first].z + m_model.getVertices()[proced[i].second].z) / 2;

					redMesh.getVertices().push_back(tris.v1());
					redMesh.getVertices().push_back(tris.v2());
					redMesh.getVertices().push_back(m_model.getVertices()[proced[i].first]);
					index = redMesh.getVertices().size() - 3;
					redMesh.getTriangles().push_back(vvr::Triangle(&redMesh.getVertices(), index, index + 1, index + 2));
					//////////////////////
				}
				else if (tris.v[2] == proced[i].second) {
					int intex1 = tris.v[1];
					int intex2 = tris.v[0];
					m_model.getTriangles().erase(m_model.getTriangles().begin() + j);
					j--;

					

					 m_model.getTriangles().push_back(vvr::Triangle(&m_model.getVertices(), proced[i].first, intex1, intex2));
					//////////////////
					m_model.getVertices()[proced[i].first].x = (m_model.getVertices()[proced[i].first].x + m_model.getVertices()[proced[i].second].x) / 2;
					m_model.getVertices()[proced[i].first].y = (m_model.getVertices()[proced[i].first].y + m_model.getVertices()[proced[i].second].y) / 2;
					m_model.getVertices()[proced[i].first].z = (m_model.getVertices()[proced[i].first].z + m_model.getVertices()[proced[i].second].z) / 2;

					redMesh.getVertices().push_back(tris.v1());
					redMesh.getVertices().push_back(tris.v2());
					redMesh.getVertices().push_back(m_model.getVertices()[proced[i].first]);
					index = redMesh.getVertices().size() - 3;
					redMesh.getTriangles().push_back(vvr::Triangle(&redMesh.getVertices(), index, index + 1, index + 2));
					//////////////////////
				}

			//	 

		//	}
			
			//m_model.getVertices()[proced[i].first].x = (m_model.getVertices()[proced[i].first].x + m_model.getVertices()[proced[i].second].x) / 2;
			//m_model.getVertices()[proced[i].first].y = (m_model.getVertices()[proced[i].first].y + m_model.getVertices()[proced[i].second].y) / 2;
			//m_model.getVertices()[proced[i].first].z = (m_model.getVertices()[proced[i].first].z + m_model.getVertices()[proced[i].second].z) / 2;


			// m_model.getVertices()[proced[i].second].x = 228360;
			 //m_model.getVertices()[proced[i].second].y = 228360;
			 //m_model.getVertices()[proced[i].second].z = 228360;


			 



	//	}



	//}
		
		
		
	//	}
		*/
		
		m_model.update();

		 
	for (int i = 0; i < m_model.getTriangles().size(); i++) {
	
		vvr::Triangle tris = m_model.getTriangles()[i];
		//Triangle3D trig;
		if (abs(embadon(Triangle3D(tris.v1().x, tris.v1().y, tris.v1().z, tris.v2().x, tris.v2().y, tris.v2().z, tris.v3().x, tris.v3().y, tris.v3().z) ))< 0.0001) {

			   m_model.getTriangles().erase(m_model.getTriangles().begin() + i);
			   i--;
		}	
	}
 
	


	for (int i = 0; i < m_model.getTriangles().size(); i++) {

		vvr::Triangle trig = m_model.getTriangles()[i];
		vec a= trig.v1();
		vec b = trig.v2();
		vec c = trig.v3();
		a.x = a.x + 50;
		b.x = b.x + 50;
		c.x = c.x + 50;


		simplified.getVertices().push_back(a);
		simplified.getVertices().push_back(b);
		simplified.getVertices().push_back(c);
		int index = simplified.getVertices().size() - 3;
		simplified.getTriangles().push_back(vvr::Triangle(&simplified.getVertices(), index, index + 1, index + 2));

	}

	//echo(m_model.getTriangles().size());
	m_model.update();
	
}
void Mesh3DScene::Convex3D() {
	 
	piramida();
	

	std::vector<vvr::Triangle3D>  trigwna_piramidas_duplicate= trigwna_piramidas;
	vector<vec> simeia_kurtou_periblimatos= piramida_points;
	

	 float x = 0;
	 float y = 0;
	 float z = 0;
	 vec cm;
	// int size = simeia.size();
	 for (int i = 0; i < piramida_points.size(); i++) {
		 x = x + piramida_points[i].x;
		 y = y + piramida_points[i].y;
		 z = z + piramida_points[i].z;

	 }
	 cm.x =	x/float(piramida_points.size());
	 cm.z =	y/float(piramida_points.size());
	 cm.y =	z/float(piramida_points.size());

	 for (int i = 0; i < m_model.getVertices().size(); i++) {

		// m_model.getVertices()[i].x =10-  m_model.getVertices()[i].x;
		// m_model.getVertices()[i].y = 10 - m_model.getVertices()[i].y;
	   	// m_model.getVertices()[i].z = 10 - m_model.getVertices()[i].z;
	 }

	// cm_piramidas.x = cm.x;
	// cm_piramidas.y = cm.y;
	// cm_piramidas.z = cm.z;

	 vector<vec> simeia = m_model.getVertices();
 
	vector<pair<int, int>> grafos;
	vec start; start.x = 0; start.y = 0; start.z = 0;
	int metr = 0;
	for (int i = 0; i < simeia.size(); i++) {
		
		for (int j = 0; j < trigwna_piramidas.size(); j++) {

			vec a1, b1, c1;
			a1.x = trigwna_piramidas[j].x1; a1.y = trigwna_piramidas[j].y1; a1.z = trigwna_piramidas[j].z1;
			b1.x = trigwna_piramidas[j].x2; b1.y = trigwna_piramidas[j].y2; b1.z = trigwna_piramidas[j].z2;
			c1.x = trigwna_piramidas[j].x3; c1.y = trigwna_piramidas[j].y3; c1.z = trigwna_piramidas[j].z3;
			Plane plane1 = Plane(a1, b1, c1);
			if ((simeia[i].x == a1.x && simeia[i].y == a1.y && simeia[i].z == a1.z) || (simeia[i].x == b1.x && simeia[i].y == b1.y && simeia[i].z == b1.z) || (simeia[i].x == c1.x && simeia[i].y == c1.y && simeia[i].z == c1.z)) { metr = 10; }
			if (plane1.d < 0) {
				plane1.normal.x = -plane1.normal.x;
				plane1.normal.y = -plane1.normal.y;
				plane1.normal.z = -plane1.normal.z;
				plane1.d = -plane1.d;
			}
			float distance1 = Dot(plane1.normal, simeia[i]) - plane1.d;;
			if (distance1 < 0) {
				metr++;
			}

		}

			if (metr >= trigwna_piramidas.size()) { 
			}
			else { pointset.push_back(simeia[i]); }
			metr = 0;
	}

	//sunolo_sigrousis_simeiwn.resize(pointset.size());
	//sunolo_sigrousis_trigwnwn.resize(trigwna_piramidas.size());
	for (int i = 0; i < pointset.size(); i++) {
		int metr = 0;
	     for (int j = 0; j < trigwna_piramidas.size(); j++) {
			vec a1, b1, c1;
			a1.x = trigwna_piramidas[j].x1; 
			a1.y = trigwna_piramidas[j].y1; 
			a1.z = trigwna_piramidas[j].z1;
			
			b1.x = trigwna_piramidas[j].x2;
			b1.y = trigwna_piramidas[j].y2;
			b1.z = trigwna_piramidas[j].z2;
			
			c1.x = trigwna_piramidas[j].x3;
			c1.y = trigwna_piramidas[j].y3;
			c1.z = trigwna_piramidas[j].z3;
			
			Plane plane1 = Plane(a1, b1, c1);
			if (plane1.d <= 0) {
				plane1.normal.x = -plane1.normal.x;
				plane1.normal.y = -plane1.normal.y;
				plane1.normal.z = -plane1.normal.z;
				plane1.d = -plane1.d;
			}
			float distance1 = Dot(plane1.normal, pointset[i]) - plane1.d;
			if (distance1 > 0) {
				grafos.push_back(make_pair(i, j));
				if (j == 0) {
					//pointset3.push_back(pointset[i]);
					//echo(i);
				}
				//sunolo_sigrousis_trigwnwn[j].push_back(i);
				//sunolo_sigrousis_simeiwn[i].push_back(j);
			}
		}
	}

	//(((   v1.x == mesh_tri.v1.x && v1.y == mesh_tri.v1.y && v1.z == mesh_tri.v1.z) || (v1.x == mesh_tri.v2.x && v1.y == mesh_tri.v2.y && v1.z == mesh_tri.v2.z) || (v1.x == mesh_tri.v3.x && v1.y == mesh_tri.v3.y && v1.z == mesh_tri.v3.z))
	//					&& ((v2.x == mesh_tri.v1.x && v2.y == mesh_tri.v1.y && v2.z == mesh_tri.v1.z) || (v2.x == mesh_tri.v2.x && v2.y == mesh_tri.v2.y && v2.z == mesh_tri.v2.z) || (v2.x == mesh_tri.v3.x && v2.y == mesh_tri.v3.y && v2.z == mesh_tri.v3.z))
	//					&& ((v3.x == mesh_tri.v1.x && v3.y == mesh_tri.v1.y && v3.z == mesh_tri.v1.z) || (v3.x == mesh_tri.v2.x && v3.y == mesh_tri.v2.y && v3.z == mesh_tri.v2.z) || (v3.x == mesh_tri.v3.x && v3.y == mesh_tri.v3.y && v3.z == mesh_tri.v3.z)))
	int size = pointset.size();
	//convexhull.getVertices().push_back(pointset[ex]);
	//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	for (int i = 0; i <pointset.size(); i++) {//pointset.size(); i++) {//0; i++) {//
		//echo(i);
		
		vector<int> intex;
		int intex_simeiou_pointset = i;
		search_grafo_for_sigrsouseis(grafos, intex,intex_simeiou_pointset);
		//briskoume se poies pleures blepei to pointset[i] simeio //
		if (intex.size() != 0) {
			//simeia_kurtou_periblimatos.push_back(pointset[i]);
			//diagrafoume apo to mesh olla ta fwtina trigwna//
			//std::vector<vec> temp_simei = convexhull.getVertices();
			/*for (int l = 0; l < convexhull.getTriangles().size(); l++) {
				vvr::Triangle mesh_tri= convexhull.getTriangles()[l];

				for (int m = 0; m < intex.size(); m++) {
					Triangle3D temp = trigwna_piramidas[intex[m]];
					vec v1,v2,v3;  
					v1.x = temp.x1;
					v1.y = temp.y1;
					v1.z = temp.z1;

					v2.x = temp.x2;
					v2.y = temp.y2;
					v2.z = temp.z2;
					
					v3.x = temp.x3;
					v3.y = temp.y3;
					v3.z = temp.z3;
					if (((   v1.x ==temp_simei[ mesh_tri.vi1].x && v1.y ==temp_simei[ mesh_tri.vi1].y && v1.z ==temp_simei[ mesh_tri.vi1].z) || (v1.x ==temp_simei[ mesh_tri.vi2].x && v1.y ==temp_simei[ mesh_tri.vi2].y && v1.z ==temp_simei[ mesh_tri.vi2].z) || (v1.x == temp_simei[ mesh_tri.vi3].x && v1.y == temp_simei[ mesh_tri.vi3].y && v1.z == temp_simei[ mesh_tri.vi3].z))
						&& ((v2.x ==temp_simei[ mesh_tri.vi1].x && v2.y ==temp_simei[ mesh_tri.vi1].y && v2.z ==temp_simei[ mesh_tri.vi1].z) || (v2.x ==temp_simei[ mesh_tri.vi2].x && v2.y ==temp_simei[ mesh_tri.vi2].y && v2.z ==temp_simei[ mesh_tri.vi2].z) || (v2.x == temp_simei[ mesh_tri.vi3].x && v2.y == temp_simei[ mesh_tri.vi3].y && v2.z == temp_simei[ mesh_tri.vi3].z))
						&& ((v3.x ==temp_simei[ mesh_tri.vi1].x && v3.y ==temp_simei[ mesh_tri.vi1].y && v3.z ==temp_simei[ mesh_tri.vi1].z) || (v3.x ==temp_simei[ mesh_tri.vi2].x && v3.y ==temp_simei[ mesh_tri.vi2].y && v3.z ==temp_simei[ mesh_tri.vi2].z) || (v3.x == temp_simei[ mesh_tri.vi3].x && v3.y == temp_simei[ mesh_tri.vi3].y && v3.z == temp_simei[ mesh_tri.vi3].z))) {
						convexhull.getTriangles().erase(convexhull.getTriangles().begin() + l);
						l--;
						//break;
					}
				
				}
			}*/
		
			//convexhull.update();
			//euresi oriwn fwtinis perioxis//
			std::vector<LineSeg3D> oria_fwtinis_perioxis;
			for (int j = 0; j < intex.size(); j++) {
				Triangle3D temp = trigwna_piramidas[intex[j]];
				oria_fwtinis_perioxis.push_back(LineSeg3D(temp.x1, temp.y1, temp.z1, temp.x2, temp.y2, temp.z2));
				oria_fwtinis_perioxis.push_back(LineSeg3D(temp.x1, temp.y1, temp.z1, temp.x3, temp.y3, temp.z3));
				oria_fwtinis_perioxis.push_back(LineSeg3D(temp.x2, temp.y2, temp.z2, temp.x3, temp.y3, temp.z3));
			}
			Euresi_path(oria_fwtinis_perioxis);
			if (oria_fwtinis_perioxis.size() == 2) {
			
				int ka = 0;
				ka = 1;
			
			}
			for (int k = 0; k < oria_fwtinis_perioxis.size(); k++) {

				vec arxi;
				arxi.x = oria_fwtinis_perioxis[k].x1;
				arxi.y = oria_fwtinis_perioxis[k].y1;
				arxi.z = oria_fwtinis_perioxis[k].z1;

				vec telos;
				telos.x = oria_fwtinis_perioxis[k].x2;
				telos.y = oria_fwtinis_perioxis[k].y2;
				telos.z = oria_fwtinis_perioxis[k].z2;
				Triangle3D neo_trig= (Triangle3D(arxi.x, arxi.y, arxi.z, telos.x, telos.y, telos.z, pointset[i].x, pointset[i].y, pointset[i].z));
				
				//edw briskw tis propsiptouses//
				vector<int> intexes_temp;
				for (int o = 0; o < trigwna_piramidas.size() - k; o++) {//-k giati kanw pushback to neo trigwno gia na exei intex kai na mpei ston grafo
					Triangle3D temp = trigwna_piramidas[o];
					if (temp.x1 == 0 && temp.y1 == 0 && temp.z1 == 0 && temp.x2 == 0 && temp.y2 == 0 && temp.z2 == 0 && temp.x3 == 0 && temp.y3 == 0 && temp.z3 == 0) {
					}
					else{
					LineSeg3D line1 = LineSeg3D(temp.x1, temp.y1, temp.z1, temp.x2, temp.y2, temp.z2);
					LineSeg3D line2 = LineSeg3D(temp.x1, temp.y1, temp.z1, temp.x3, temp.y3, temp.z3);
					LineSeg3D line3 = LineSeg3D(temp.x2, temp.y2, temp.z2, temp.x3, temp.y3, temp.z3);
					if ((line1.x1 == arxi.x && line1.y1 == arxi.y && line1.z1 == arxi.z && line1.x2 == telos.x && line1.y2 == telos.y && line1.z2 == telos.z) || (line1.x1 == telos.x && line1.y1 == telos.y && line1.z1 == telos.z && line1.x2 == arxi.x && line1.y2 == arxi.y && line1.z2 == arxi.z)) {
						intexes_temp.push_back(o);
					}
					if ((line2.x1 == arxi.x && line2.y1 == arxi.y && line2.z1 == arxi.z && line2.x2 == telos.x && line2.y2 == telos.y && line2.z2 == telos.z) || (line2.x1 == telos.x && line2.y1 == telos.y && line2.z1 == telos.z && line2.x2 == arxi.x && line2.y2 == arxi.y && line2.z2 == arxi.z)) {
						intexes_temp.push_back(o);
					}
					if ((line3.x1 == arxi.x && line3.y1 == arxi.y && line3.z1 == arxi.z && line3.x2 == telos.x && line3.y2 == telos.y && line3.z2 == telos.z) || (line3.x1 == telos.x && line3.y1 == telos.y && line3.z1 == telos.z && line3.x2 == arxi.x && line3.y2 == arxi.y && line3.z2 == arxi.z)) {
						intexes_temp.push_back(o);
					}
				}
				}

				 Triangle3D trig1 = trigwna_piramidas[intexes_temp[0]];
				 Triangle3D trig2 = trigwna_piramidas[intexes_temp[1]];
			
				 vec v1, v2, v3;
				 v1.x = trig1.x1;v1.y = trig1.y1;v1.z = trig1.z1;v2.x = trig1.x2; v2.y = trig1.y2;v2.z = trig1.z2;v3.x = trig1.x3;v3.y = trig1.y3;v3.z = trig1.z3;

				 Plane sinepipeda1 = Plane(v1, v2, v3);
				 
				 v1.x = trig2.x1;v1.y = trig2.y1;v1.z = trig2.z1; v2.x = trig2.x2;v2.y = trig2.y2;v2.z = trig2.z2;v3.x = trig2.x3;v3.y = trig2.y3;v3.z = trig2.z3;
				 Plane sinepipeda2 = Plane(v1, v2, v3);

				if (Cross(sinepipeda1.normal,sinepipeda2.normal)[0] == 0 && Cross(sinepipeda1.normal, sinepipeda2.normal)[1]==0 && Cross(sinepipeda1.normal, sinepipeda2.normal)[2]==0) {
				
					int ka = 0;
						ka = 8;
				
				}
				else {
					vector<int> intex_simeia_enwsis;
					for (int p = 0; p < grafos.size(); p++) {
						//for (int in = 0; in < intexes_temp.size(); in++) {
							//if (grafos[p].second == intexes_temp[in]) {
							//	intex_simeia_enwsis.push_back(grafos[p].first);
						//	}
					//	}
						
						if (grafos[p].second == intexes_temp[0]) {
							intex_simeia_enwsis.push_back(grafos[p].first);
						}
						
						if (grafos[p].second == intexes_temp[1]) {
								intex_simeia_enwsis.push_back(grafos[p].first);
							}
						
					}
					vector<pair<int, int>>  temp_grafos;
					//valid_trigwna_piramidas.push_back(make_pair(neo_trig,0));
					trigwna_piramidas.push_back(neo_trig);
					Plane plane1 = Plane( arxi, telos, pointset[i]);
					//if ((simeia[i].x == a1.x && simeia[i].y == a1.y && simeia[i].z == a1.z) || (simeia[i].x == b1.x && simeia[i].y == b1.y && simeia[i].z == b1.z) || (simeia[i].x == c1.x && simeia[i].y == c1.y && simeia[i].z == c1.z)) { metr = 10; }
					if (plane1.d < 0) {
						plane1.normal.x = -plane1.normal.x;
						plane1.normal.y = -plane1.normal.y;
						plane1.normal.z = -plane1.normal.z;
						plane1.d =  -plane1.d;
					} 
					//for (int t = 0; t < intex_simeia_enwsis.size(); t++) {
					for (int t = 0; t < intex_simeia_enwsis.size(); t++) {
						float distance1 = Dot(plane1.normal, pointset[intex_simeia_enwsis[t]])-  plane1.d ;
						if (distance1 > 0) {
							temp_grafos.push_back(make_pair(intex_simeia_enwsis[t], trigwna_piramidas.size() - 1));
						}
					
					}


					for (int temi = 0;temi < temp_grafos.size();temi++) {
						for (int temj = 0; temj < temp_grafos.size(); temj++) {
							if (temi != temj) {
								if (temp_grafos[temi].first == temp_grafos[temj].first && temp_grafos[temi].second == temp_grafos[temj].second) {

									temp_grafos[temj].first = -1;
									temp_grafos[temj].second = -1;
								}
							}
						}
					}
					for (int temi = 0; temi < temp_grafos.size(); temi++) {
						if (temp_grafos[temi].first == -1 && temp_grafos[temi].second == -1) {
							temp_grafos.erase(temp_grafos.begin() + temi);
							temi--;
						}
					}

					for (int temi = 0; temi < temp_grafos.size(); temi++) {
					
						grafos.push_back(temp_grafos[temi]);
					
					}

					/*
					for (int t = 0; t < pointset.size(); t++) {
						float distance1 = Dot(plane1.normal, pointset[t]) - plane1.d;;
						if (distance1 >0) {
							grafos.push_back(make_pair(t, trigwna_piramidas.size() - 1));
						}

					}
					*/

				}
			
			/*	int tempo2 = 0;
				for (int ki = 0; ki <intex.size() ; ki++) {
					if (intex[ki] == intexes_temp[0]) {
						tempo2 = intex[ki]; break;
					}
				
					if (intex[ki] == intexes_temp[1]) {
						tempo2 = intex[ki]; break;
					}
				
				
				}
				for (int q = 0; q < grafos.size(); q++) {
					if (grafos[q].second ==  tempo2) {
						//grafos[q].second = 0;
						grafos.erase(grafos.begin() + q);
						q--;// break;
					}
				}*/
			}
			for (int e = 0; e < intex.size(); e++) {
				trigwna_piramidas[intex[e]].x1 = 0;
				trigwna_piramidas[intex[e]].y1 = 0;
				trigwna_piramidas[intex[e]].z1 = 0;

				trigwna_piramidas[intex[e]].x2 = 0;
				trigwna_piramidas[intex[e]].y2 = 0;
				trigwna_piramidas[intex[e]].z2 = 0;

				trigwna_piramidas[intex[e]].x3 = 0;
				trigwna_piramidas[intex[e]].y3 = 0;
				trigwna_piramidas[intex[e]].z3 = 0;
			}

			for (int e = 0; e < oria_fwtinis_perioxis.size(); e++) {
				Triangle3D temp= trigwna_piramidas[trigwna_piramidas.size()-1-e];
				vec a, b, c;
				a.x = temp.x1;
				a.y = temp.y1;
				a.z = temp.z1;

				b.x = temp.x2;
				b.y = temp.y2;
				b.z = temp.z2;
				
				c.x = temp.x3;
				c.y = temp.y3;
				c.z = temp.z3;	
				convexhull.getVertices().push_back(a);
				convexhull.getVertices().push_back(b);
				convexhull.getVertices().push_back(c);

			//	int index = convexhull.getVertices().size() - 3;
			//	convexhull.getTriangles().push_back(vvr::Triangle(&convexhull.getVertices(), index, index + 1, index + 2));
			}
			
			for (int q = 0; q < grafos.size(); q++) {
				if (grafos[q].first == i) {
					grafos.erase(grafos.begin() + q);
					q--;
				}
			}

			for (int w = 0; w < intex.size(); w++) {	
				for (int q = 0; q < grafos.size(); q++) {
					if (grafos[q].second == intex[w]) {
						grafos[q].second = 0;
						//grafos.erase(grafos.begin() + q);
						//q--;// break;
					}
				}

			}
			for (int q = 0; q < grafos.size(); q++) {
				if (grafos[q].second == 0) {
					grafos.erase(grafos.begin() + q);
				
					q--;
				}
			
			
			}
		}
	}
	
	for (int i = 0; i < trigwna_piramidas.size(); i++) {

		if (trigwna_piramidas[i].x1 == 0 &&
			trigwna_piramidas[i].y1 == 0 &&
			trigwna_piramidas[i].z1 == 0 && trigwna_piramidas[i].x2 == 0 && trigwna_piramidas[i].y2 == 0 &&
			trigwna_piramidas[i].z2 == 0 && trigwna_piramidas[i].x3 == 0 &&
			trigwna_piramidas[i].y3 == 0 &&
			trigwna_piramidas[i].z3 == 0) {
		}
		else {
			vec temp1;
			temp1.x = trigwna_piramidas[i].x1;
			temp1.y = trigwna_piramidas[i].y1;
			temp1.z = trigwna_piramidas[i].z1;
			akraia_simeia.push_back(temp1);

			vec temp2;
			temp2.x = trigwna_piramidas[i].x2;
			temp2.y = trigwna_piramidas[i].y2;
			temp2.z = trigwna_piramidas[i].z2;

			akraia_simeia.push_back(temp2);
			vec temp3;
			temp3.x = trigwna_piramidas[i].x3;
			temp3.y = trigwna_piramidas[i].y3;
			temp3.z = trigwna_piramidas[i].z3;
			akraia_simeia.push_back(temp3);

		//	akraia_simeia.push_back(vec((temp1.x + temp2.x + temp3.x) / 3, (temp1.y + temp2.y + temp3.y) / 3, (temp1.z + temp2.z + temp3.z) / 3));


		}
	}



	for (int i = 0; i < akraia_simeia.size(); i++) {
		for (int j = i; j < akraia_simeia.size(); j++) {
			if (i != j) {

				if (akraia_simeia[i].x ==akraia_simeia[j].x && akraia_simeia[i].y == akraia_simeia[j].y && akraia_simeia[i].z == akraia_simeia[j].z) {

					akraia_simeia[j].x = inf;
					akraia_simeia[j].y = inf;
					akraia_simeia[j].z = inf;



				}

			}

		}

	}


	for (int i = 0; i < akraia_simeia.size(); i++) {
		if (akraia_simeia[i].x == inf && akraia_simeia[i].y == inf && akraia_simeia[i].z == inf) {

			akraia_simeia.erase(akraia_simeia.begin() + i);
			i--;
		}

	}

	 

}
void Mesh3DScene::search_grafo_for_sigrsouseis(std::vector<pair<int, int>> grafos, vector<int> &intex, int intex_simeiou_pointset) {


	for (int i = 0; i < grafos.size(); i++) {
		if (intex_simeiou_pointset == grafos[i].first)
		{
			intex.push_back(grafos[i].second);

		}
	}
/* 
	for (int i = 0; i < intex.size(); i++) {
		for (int j = 0; j < intex.size(); j++) {
			if (i != j) {
				if (intex[i] == intex[j]) {
				
					intex[j] = -1;
				}
			
			
			}
		}


	}



	for (int i = 0; i < intex.size(); i++) {

			if (intex[i] == -1) {

				intex.erase(intex.begin() + i);
				i--;
		}


	}

	*/

	/*
	for (int i = 0; i < grafos.size(); i++) {
		if(point.x==pointset[grafos[i].first].x && point.y == pointset[grafos[i].first].y && point.z == pointset[grafos[i].first].z)
		{
		
			intex.push_back(grafos[i].second);
		
		}
	
	
	
	}
	*/
	/*
	int size = intex2.size() - 1;
	for (int i = 0; i < size; i++)
		{
			if (intex2[i] == intex2[i+1]) intex2[i + 1] = inf;
			

			}

	for (int i = 0; i < intex2.size() ; i++)
	{
		if (intex2[i] != inf) {


			intex.push_back(intex2[i]);


		}
	}
	*/
}
void Mesh3DScene::Euresi_path(std::vector<LineSeg3D> & path) {
	std::vector<LineSeg3D> temp;
	temp = path;

	//if (path.size() == 3)return;

	for (int i = 0; i < path.size(); i++) {
		for (int j = i; j < path.size(); j++) {
			if (i != j) {
				if ((path[i].x1 == path[j].x1	&& path[i].y1 == path[j].y1 && path[i].z1 == path[j].z1 && path[i].x2 == path[j].x2	&& path[i].y2 == path[j].y2 && path[i].z2 == path[j].z2) || (path[i].x1 == path[j].x2	&& path[i].y1 == path[j].y2 && path[i].z1 == path[j].z2 && path[i].x2 == path[j].x1	&& path[i].y2 == path[j].y1 && path[i].z2 == path[j].z1)) {

					path[i].x1 = inf;
					path[i].y1 = inf;
					path[i].z1 = inf;

					path[i].x2 = inf;
					path[i].y2 = inf;
					path[i].z2 = inf;

					path[j].x1 = inf;
					path[j].y1 = inf;
					path[j].z1 = inf;

					path[j].x2 = inf;
					path[j].y2 = inf;
					path[j].z2 = inf;
					

				}

			}


		}


	}

	for (int i = 0; i < path.size(); i++) {
		if (path[i].x1 == inf && path[i].y1 == inf && path[i].z1 == inf &&	path[i].x2 == inf &&path[i].y2 == inf && path[i].z2 == inf) {
			path.erase(path.begin() + i);
			i--;
		}
	
	
	}





	/*
	for (int i = 0; i < path.size(); i++) {
		bool fl = false;
		for (int j =i; j < path.size(); j++) {
			if(i!=j){
			if ((path[i].x1 == path[j].x1	&& path[i].y1 == path[j].y1 && path[i].z1 == path[j].z1 && path[i].x2 == path[j].x2	&& path[i].y2 == path[j].y2 && path[i].z2 == path[j].z2) || (path[i].x1 == path[j].x2	&& path[i].y1 == path[j].y2 && path[i].z1 == path[j].z2 && path[i].x2 == path[j].x1	&& path[i].y2 == path[j].y1 && path[i].z2 == path[j].z1)) {
				
				
				 fl = true;
				  path.erase(path.begin() + j);
				
				  j--;
				 path.erase(path.begin() + i);
				 j--;
			}

		}
		
		
		}
		if(fl==true)i--;
		//flag = false;
	
	
	}


	*/



}
void Mesh3DScene::piramida() {
	
	for (int i = 0; i < m_model.getVertices().size(); i++) {


		float z= m_model.getVertices()[i].z;
		float x=m_model.getVertices()[i].x;


		 m_model.getVertices()[i].z = z*cos(45)- x*sin(45);
		 m_model.getVertices()[i].x = z*sin(45) + x*cos(45);

	}
	vvr::Mesh  mesh = m_model;
	 std::vector<vec>  temp_points=mesh.getVertices();
	
	//std::vector<vec>  temp_points = kentra_dual_graph;

	float minx = temp_points[1].x;
	float miny = temp_points[1].y;
	float minz = temp_points[1].z;
	float maxx = temp_points[1].x;
	float maxy = temp_points[1].y;
	float maxz = temp_points[1].z;
	vec temp1, temp2, temp3, temp4, temp5, temp6;

	for (int i = 0; i < temp_points.size(); i++) {

		if (temp_points[i].x < minx) { minx = temp_points[i].x;  temp1.x = temp_points[i].x; temp1.y = temp_points[i].y; temp1.z = temp_points[i].z;}
		if (temp_points[i].y < miny) { miny = temp_points[i].y;  temp2.x = temp_points[i].x; temp2.y = temp_points[i].y; temp2.z = temp_points[i].z;}
		if (temp_points[i].z < minz) { minz = temp_points[i].z;  temp3.x = temp_points[i].x; temp3.y = temp_points[i].y; temp3.z = temp_points[i].z;}
		if (temp_points[i].x > maxx) { maxx = temp_points[i].x;  temp4.x = temp_points[i].x; temp4.y = temp_points[i].y; temp4.z = temp_points[i].z;}
		if (temp_points[i].y > maxy) { maxy = temp_points[i].y;  temp5.x = temp_points[i].x; temp5.y = temp_points[i].y; temp5.z = temp_points[i].z;}
		if (temp_points[i].z > maxz) { maxz = temp_points[i].z;  temp6.x = temp_points[i].x; temp6.y = temp_points[i].y; temp6.z = temp_points[i].z;}


	}
	piramida_points.push_back(temp1);
	piramida_points.push_back(temp2);
	piramida_points.push_back(temp3);
	piramida_points.push_back(temp4);
	piramida_points.push_back(temp5);
	piramida_points.push_back(temp6);
	float max=0;
	vec point1, point2;
	for (int i=0; i < piramida_points.size(); i++) {
		for (int j =0; j < piramida_points.size(); j++) {
			if (piramida_points[i].Distance(piramida_points[j]) > max) {
				max = piramida_points[i].Distance(piramida_points[j]);
				point1.x = piramida_points[i].x; point1.y = piramida_points[i].y; point1.z = piramida_points[i].z;
				point2.x = piramida_points[j].x; point2.y = piramida_points[j].y; point2.z = piramida_points[j].z;
			
			
			
			}
		}
	}
	for (int i = 0; i <6; i++) { piramida_points.pop_back(); }
	
	piramida_points.push_back(point1);
	piramida_points.push_back(point2);
	max = 0;
	vec point3;
	
	
	for (int i = 0; i < temp_points.size(); i++) {
		vec x0 = temp_points[i];
		vec x1 = point1;
		vec x2 = point2;
		vec temp1,temp2,temp3;
		temp1.x = x1.x - x0.x;
		temp1.y = x1.y - x0.y;
		temp1.z = x1.z - x0.z;
		//vec v;
		temp2.x = x2.x - x1.x;
		temp2.y = x2.y - x1.y;
		temp2.z = x2.z - x1.z;


		Dot(point1, point2);

		float metro2 = pow(temp2.x, 2) + pow(temp2.y, 2) + pow(temp2.z, 2);
		float metro1 = pow(temp1.x, 2) + pow(temp1.y, 2) + pow(temp1.z, 2);
		float metro3 = pow(Dot(temp1, temp2), 2);


		float distance =(metro2*metro1-metro3) / metro2 ;
		distance = pow(distance, 0.5);
		if (distance > max) {
			max = distance;
			point3.x = temp_points[i].x;
			point3.y = temp_points[i].y;
			point3.z = temp_points[i].z;
		}
	}

	piramida_points.push_back(point3);
 
	Plane plane=Plane(point1, point2, point3);



		max = 0;
		vec point4;
	for (int i = 0; i < temp_points.size(); i++) {
	
	
		vec temp1, temp2, temp3;
		temp1 = temp_points[i];
		temp2.x = point2.x - point1.x;
		temp2.y = point2.y - point1.y;
		temp2.z = point2.z - point1.z;

		temp3.x = point3.x - point1.x;
		temp3.y = point3.y - point1.y;
		temp3.z = point3.z - point1.z;
		vec cros= Cross(temp3,temp1);
		float metr_cros = pow(pow(cros.x, 2) + pow(cros.y, 2) + pow(cros.z, 2), 0.5);
		vec n;
		n.x = cros.x / metr_cros;
		n.y = cros.y / metr_cros;
		n.z = cros.z / metr_cros;
 
		
		
		float distance =Dot(plane.normal, temp1)-plane.d;

		if (distance > max) {
			max = distance;
			point4.x = temp_points[i].x;
			point4.y = temp_points[i].y;
			point4.z = temp_points[i].z;
		}

	}



	piramida_points.push_back(point4);


	float x = 0;
	float y = 0;
	float z = 0;
	vec cm;
	int size = piramida_points.size();
	for (int i = 0; i < piramida_points.size(); i++) {
		x = x + piramida_points[i].x;
		y = y + piramida_points[i].y;
		z = z + piramida_points[i].z;

	}
	cm.x = x / float(size);
	cm.z = z / float(size);
	cm.y = y / float(size);



	for (int i = 0; i < temp_points.size(); i++) {

		 m_model.getVertices()[i].x = -cm.x + m_model.getVertices()[i].x;
		 m_model.getVertices()[i].y = -cm.y + m_model.getVertices()[i].y;
		 m_model.getVertices()[i].z = -cm.z + m_model.getVertices()[i].z;
	}



	for (int i = 0; i < piramida_points.size(); i++) {
		piramida_points[i].x = -cm.x + piramida_points[i].x;
		piramida_points[i].y = -cm.y + piramida_points[i].y;
		piramida_points[i].z = -cm.z + piramida_points[i].z;
	
	
	
	}
	
	 trigwna_piramidas.push_back( Triangle3D(piramida_points[0].x, piramida_points[0].y, piramida_points[0].z, piramida_points[1].x, piramida_points[1].y, piramida_points[1].z, piramida_points[2].x, piramida_points[2].y, piramida_points[2].z));
	 trigwna_piramidas.push_back(Triangle3D(piramida_points[0].x, piramida_points[0].y, piramida_points[0].z, piramida_points[1].x, piramida_points[1].y, piramida_points[1].z, piramida_points[3].x, piramida_points[3].y, piramida_points[3].z));
	 trigwna_piramidas.push_back(Triangle3D(piramida_points[3].x, piramida_points[3].y, piramida_points[3].z, piramida_points[1].x, piramida_points[1].y, piramida_points[1].z, piramida_points[2].x, piramida_points[2].y, piramida_points[2].z));
	 trigwna_piramidas.push_back(Triangle3D(piramida_points[0].x, piramida_points[0].y, piramida_points[0].z, piramida_points[3].x, piramida_points[3].y, piramida_points[3].z, piramida_points[2].x, piramida_points[2].y, piramida_points[2].z));




	 convexhull.getVertices().push_back(piramida_points[0]);
	 convexhull.getVertices().push_back(piramida_points[1]);
	 convexhull.getVertices().push_back(piramida_points[2]);
	 convexhull.getVertices().push_back(piramida_points[3]);
	 convexhull.getTriangles().push_back(vvr::Triangle(&convexhull.getVertices(), 0, 1, 2));
	 convexhull.getTriangles().push_back(vvr::Triangle(&convexhull.getVertices(), 0, 1, 3));
	 convexhull.getTriangles().push_back(vvr::Triangle(&convexhull.getVertices(), 1, 2, 3));
	 convexhull.getTriangles().push_back(vvr::Triangle(&convexhull.getVertices(), 0, 2, 3));



}
void Mesh3DScene::Dijkstra(int src, float &max,float &min ) {
	 
	std::vector<vvr::Triangle> &trigwna = m_model_original.getTriangles();
	vector<float> distances(kentra_dual_graph.size(), inf);
	distances[src] = 0;
	 
	priority_queue< iPair, vector <iPair>, compare > pq;
	pq.push(make_pair(float(0), src));
	
	while (!pq.empty()) {
		int u = pq.top().second;
		pq.pop();
		for (int i = 0; i < kanonika[u+1]- kanonika[u]; i++) {//for (int i = 0; i < 3; i++) {
			 
			 
				int v = int(graphfinal2[kanonika[u]+ i][1]);//int v = int(graphfinal2[3 * u + i][1]);//int v = int(graphfinal2[3 * u + i][1]);
				float weight = graphfinal2[kanonika[u] + i][2];//float weight = graphfinal2[3 * u + i][2];
			if (distances[v] > distances[u] + weight) {
				distances[v] = distances[u] + weight;
				pq.push(make_pair(distances[v], v));
			}
			//edw na balw ena metriti me ena break glitwssw epanalispsesis
	 
		 
	}
	}


	float geodesic=0;

	for (int i = 0; i < kentra_dual_graph.size(); ++i) {

		float em_trig = embadon(Triangle3D(trigwna[i].v1().x, trigwna[i].v1().y, trigwna[i].v1().z, trigwna[i].v2().x, trigwna[i].v2().y, trigwna[i].v2().z, trigwna[i].v3().x, trigwna[i].v3().y, trigwna[i].v3().z));
		
		geodesic = geodesic + distances[i]*em_trig;
	}

	if (geodesic > max) { max = geodesic; }
	if (geodesic < min) { min = geodesic; }
	geodesiakes.push_back(geodesic);


}
float embadon(Triangle3D trig) {
	vec a, b, c;
	a.x = trig.x1;
	a.y = trig.y1;
	a.z = trig.z1;
	b.x = trig.x2;
	b.y = trig.y2;
	b.z = trig.z2;
	c.x = trig.x3;
	c.y = trig.y3;
	c.z = trig.z3;
	float da = a.Distance(b);
	float db = b.Distance(c);
	float dc = a.Distance(c);
	float s = (da + db + dc) / 2;
	float t = s*(s - da)*(s - db)*(s - dc);
	pow(t, 0.5);
	return pow(t, 0.5);
}
void Mesh3DScene::reeb_graph_height() {
	int kat =10;
	float bima = 1 / float(kat);
	vector<vector<int>> intex_trigwnwn_katigorias_green;
	vector<vec> simeia = m_model.getVertices();
	float max = 0;
	float min = inf;
	float minx = inf;
	float maxx = 0;
	for (int i = 0; i < simeia.size(); i++) {
		if (simeia[i].y > max)max = simeia[i].y;
		if (simeia[i].y < min)min = simeia[i].y;
	
	}

	for (int i = 0; i < m_model.getTriangles().size(); i++) {
	
		vvr::Triangle trigwno = m_model.getTriangles()[i];
		vec a = trigwno.v1();
		vec b = trigwno.v2();
		vec c = trigwno.v3();

		int kat_a = int(((a.y - min) / (max - min)) / bima);
		int kat_b = int(((b.y - min) / (max - min)) / bima);
		int kat_c = int(((c.y - min) / (max - min)) / bima);

		if (kat_a == kat_b && kat_a == kat_c) {}
		else if ((kat_a == kat_b && kat_a != kat_c) || (kat_a != kat_b && kat_a == kat_c) || (kat_c == kat_b && kat_a != kat_c)) {
			//m_model.getTriangles().erase(m_model.getTriangles().begin() + i);
			//i--;
			float plane_height;
			if (kat_a >= kat_b && kat_a >= kat_c)	   { plane_height = float(kat_a*bima*(max - min)) + min; }
			else if (kat_b >= kat_c && kat_b >= kat_c) { plane_height = float(kat_b*bima*(max - min)) + (min); }
			else if (kat_c >= kat_a && kat_c >= kat_b) { plane_height = float(kat_c*bima*(max - min)) + (min); }
			plane = Plane(vec(-50, plane_height, -50), vec(-50, plane_height, 50), vec(50, plane_height, 50));
			vec idia1;
			vec idia2;
			vec diafo;
			if (kat_a == kat_b) { idia1 = a; idia2 = b; diafo = c; }
			else if (kat_a == kat_c) { idia1 = a; idia2 = c; diafo = b; }
			else { idia1 = b; idia2 = c; diafo = a; }
			vec contact1;
			vec contact2;
			Line_Plane_Intersection(contact1, LineSeg3D(idia1.x, idia1.y, idia1.z, diafo.x, diafo.y, diafo.z), plane);
			Line_Plane_Intersection(contact2, LineSeg3D(idia2.x, idia2.y, idia2.z, diafo.x, diafo.y, diafo.z), plane);
			////gia na min exw idio contact1 kai contact2
			if (!((abs(contact1.x - diafo.x) <0.001 && abs(contact1.y - diafo.y) <0.001 && abs(contact1.z - diafo.z) <0.001 )||( abs(contact2.x - diafo.x) <0.001 && abs(contact2.y - diafo.y) <0.001 && abs(contact2.z - diafo.z) <0.001))) {
				 m_model.getTriangles().erase(m_model.getTriangles().begin() + i);
				 i--;
				 
				
				 

				greenMesh.getVertices().push_back(contact1);
				greenMesh.getVertices().push_back(contact2);
				greenMesh.getVertices().push_back(diafo);
				int index = greenMesh.getVertices().size() - 3;
				greenMesh.getTriangles().push_back(vvr::Triangle(&greenMesh.getVertices(), index, index + 1, index + 2));

				Triangle3D temp_1(idia1.x, idia1.y, idia1.z, idia2.x, idia2.y, idia2.z, contact1.x, contact1.y, contact1.z);
				Triangle3D temp_2(contact2.x, contact2.y, contact2.z, idia2.x, idia2.y, idia2.z, contact1.x, contact1.y, contact1.z);
				Triangle3D olokliro(idia1.x, idia1.y, idia1.z, idia2.x, idia2.y, idia2.z, diafo.x, diafo.y, diafo.z);
				Triangle3D monotou(contact2.x, contact2.y, contact2.z, diafo.x, diafo.y, diafo.z, contact1.x, contact1.y, contact1.z);


				greenMesh.getVertices().push_back(idia1);
				greenMesh.getVertices().push_back(idia2);

				if (abs((embadon(temp_1) + embadon(temp_2)) - abs(embadon(olokliro) - embadon(monotou))) < 0.001) {

					int index = greenMesh.getVertices().size() - 5;
					greenMesh.getTriangles().push_back(vvr::Triangle(&greenMesh.getVertices(), index, index + 3, index + 4));
					index = greenMesh.getVertices().size() - 5;
					greenMesh.getTriangles().push_back(vvr::Triangle(&greenMesh.getVertices(), index, index + 1, index + 4));

				}
				else {
					int index = greenMesh.getVertices().size() - 5;
					greenMesh.getTriangles().push_back(vvr::Triangle(&greenMesh.getVertices(), index, index + 3, index + 4));
					index = greenMesh.getVertices().size() - 5;
					greenMesh.getTriangles().push_back(vvr::Triangle(&greenMesh.getVertices(), index, index + 1, index + 3));
				}
			}
		}
	}
	vector<vector<int>> intex_trigwnwn_katigorias;
	intex_trigwnwn_katigorias.resize(kat );
	for (int i = 0; i < m_model.getTriangles().size(); i++) {

		vvr::Triangle trigwno = m_model.getTriangles()[i];
		vec a = trigwno.v1();
		vec b = trigwno.v1();
		vec c = trigwno.v1();

		int kat_a = int(((a.y - min) / (max - min)) / bima);
		int kat_b = int(((b.y - min) / (max - min)) / bima);
		int kat_c = int(((c.y - min) / (max - min)) / bima);
		if (kat_a == kat_b) {  if (kat_a == kat)kat_a = kat_a - 1; intex_trigwnwn_katigorias[kat_a].push_back(i);
		}
		else if (kat_c == kat_b) { if (kat_c == kat)kat_c = kat_c - 1; intex_trigwnwn_katigorias[kat_c].push_back(i);
		}
		else if (kat_a == kat_c) {  if (kat_a == kat)kat_a = kat_a - 1; intex_trigwnwn_katigorias[kat_a].push_back(i);
		}
	
	}

	intex_trigwnwn_katigorias_green.resize(kat);

	for (int i = 0; i < greenMesh.getTriangles().size(); i++) {

		vvr::Triangle trigwno = greenMesh.getTriangles()[i];
		vec a = trigwno.v3();

		int kat_a = int(((a.y - min) / (max - min)) / bima);
		if (kat_a == kat)kat_a = kat_a - 1;
		intex_trigwnwn_katigorias_green[kat_a].push_back(i);

	}
	vector<vector<pair<int, int>>> intexes;
	intexes.resize(kat);
	
	for (int j = 0; j < kat; j++) {
		for (int i = 0; i < intex_trigwnwn_katigorias[j].size(); i++) {
			intexes[j].push_back(make_pair(intex_trigwnwn_katigorias[j][i], 1));

		}
	}
	for (int j = 0; j < kat; j++) {
		for (int i = 0; i < intex_trigwnwn_katigorias_green[j].size(); i++) {
			intexes[j].push_back(make_pair(intex_trigwnwn_katigorias_green[j][i], 2));

		}
	}

	vector<int> katigoria;
	int metritis_antikeimenwn=0;
	//antikeimena_mesh.resize(antikeimena_mesh.size() + 1);
	int k =0;
	for (int ji =0; ji <kat; ji++) {
		vector<int> processed;
		processed.resize(intexes[ji].size(),0);
		for (int i = 0; i < processed.size(); i++) {
			if (processed[i] != 1) {
				antikeimena_mesh.resize(antikeimena_mesh.size() + 1);
				antikeimena(intexes[ji], processed, i, metritis_antikeimenwn);

				katigoria.push_back(ji);
				metritis_antikeimenwn++;
			 
			}
		}	
	}
	////edw bgazw antikeimena pou den ta 8ewrw antikeimena
	for (int j = 0; j < antikeimena_mesh.size(); j++) {
	
		if (antikeimena_mesh[j].getTriangles().size() < 4) {
			antikeimena_mesh.erase(antikeimena_mesh.begin() + j);
			katigoria.erase(katigoria.begin()+j);
			j--;
		}
	}
	////edw briskw ta kentra mazas gia to ka8e anteikimenoo
	float x = 0;
	float y = 0;
	float z = 0;
	float embad=0;
 
	cm_antikeimenwn.resize(antikeimena_mesh.size());
	for (int i = 0; i < antikeimena_mesh.size(); i++) {
		for (int j = 0; j < antikeimena_mesh[i].getTriangles().size(); j++) {

			vvr::Triangle trigwno = antikeimena_mesh[i].getTriangles()[j];
			vec a=trigwno.getCenter();
			vec b = trigwno.v1();
			vec c = trigwno.v2();
			vec d = trigwno.v3();

			 
			x = x + a.x;
			y = y + a.y;
			z = z + a.z;
		}
		 
		cm_antikeimenwn[i].x = x / (float(antikeimena_mesh[i].getTriangles().size()));
		cm_antikeimenwn[i].y = y / (float(antikeimena_mesh[i].getTriangles().size()));
		cm_antikeimenwn[i].z = z / (float(antikeimena_mesh[i].getTriangles().size()));
	 
		 x = 0;
		 y = 0;
		 z = 0;
	}

	//edw enwnw ta cm gia na ftiaksw ton reeb graph//
	for (int i = 0; i < antikeimena_mesh.size(); i++) {		
		for (int j = i; j < katigoria.size(); j++) {
			bool flag = false;
	 
				LineSeg3D line=LineSeg3D(cm_antikeimenwn[i].x, cm_antikeimenwn[i].y, cm_antikeimenwn[i].z, cm_antikeimenwn[j].x, cm_antikeimenwn[j].y, cm_antikeimenwn[j].z,vvr::Colour::darkOrange);
			
				for (int k = 0; k < antikeimena_mesh[i].getTriangles().size(); k++) {
					vvr::Triangle trig = antikeimena_mesh[i].getTriangles()[k];
					vec p1 = trig.v1();
					vec p2 = trig.v2();
					vec p3 = trig.v3();

					if (FindAdjacentTriangle(antikeimena_mesh[j], p1, p2) || FindAdjacentTriangle(antikeimena_mesh[j], p3, p2) || FindAdjacentTriangle(antikeimena_mesh[j], p3, p1)) {
						flag = true;
						break;					
					}
					}
				if (flag == true) {
					reeb_lines.push_back(line);
				 
			}
		}
	}
	 

}
void Mesh3DScene::reeb_graph_geodesic_distances() {
	 telika_akria(1);

	int kat = 10;
	float bima = 1 / float(kat);

	vector<vector<int>> intex_trigwnwn_katigorias_green;
	float max = 0;
	float min = inf;
	float minx = inf;
	float maxx = 0;
	for (int i = 0; i < geodesic_akraiwn.size(); i++) {
		if (geodesic_akraiwn[i].second > max)max = geodesic_akraiwn[i].second;
		if (geodesic_akraiwn[i].second < min)min = geodesic_akraiwn[i].second;

	}
	int g = 0;
	g = 1;

	vector<vector<int>> intex_trigwnwn_katigorias;
	intex_trigwnwn_katigorias.resize(kat+10);
	for (int i = 0; i < m_model.getTriangles().size(); i++) {

		int kat_a = int(((geodesiakes[i] - min_g) / (max_g - min_g)) / bima);
		if (kat_a == kat)kat_a = kat_a - 1;
		intex_trigwnwn_katigorias[kat_a].push_back(i);
		
	}
	vector<vector<pair<int, int>>> intexes;
	intexes.resize(kat);
	for (int j = 0; j < kat; j++) {
		for (int i = 0; i < intex_trigwnwn_katigorias[j].size(); i++) {
			intexes[j].push_back(make_pair(intex_trigwnwn_katigorias[j][i], 1));

		}
	}
	vector<int> katigoria;
	int metritis_antikeimenwn = 0;
	int k = 0;
	for (int ji = 0; ji <kat; ji++) {
		vector<int> processed;
		processed.resize(intexes[ji].size(), 0);
		for (int i = 0; i < processed.size(); i++) {
			if (processed[i] != 1) {
				antikeimena_mesh.resize(antikeimena_mesh.size() + 1);
				antikeimena(intexes[ji], processed, i, metritis_antikeimenwn);
				katigoria.push_back(ji);
				metritis_antikeimenwn++;
			}
		}
	}
	for (int j = 0; j < antikeimena_mesh.size(); j++) {

		if (antikeimena_mesh[j].getTriangles().size() < 5) {//7
			antikeimena_mesh.erase(antikeimena_mesh.begin() + j);
	

			katigoria.erase(katigoria.begin() + j);
			j--;
		}

	}
 
	////edw briskw ta kentra mazas gia to ka8e anteikimenoo
	float x = 0;
	float y = 0;
	float z = 0;
	float embad = 0;
	
	cm_antikeimenwn.resize(antikeimena_mesh.size());
	for (int i = 0; i < antikeimena_mesh.size(); i++) {
		vec temp = vec(inf, inf, inf);
		int metritis = 0;
		bool flag = false;

		for (int j = 0; j < antikeimena_mesh[i].getTriangles().size(); j++) {

			vvr::Triangle trigwno = antikeimena_mesh[i].getTriangles()[j];
			vec a = trigwno.v1();
			vec b = trigwno.v2();
			vec c = trigwno.v3();
			vec tr = trigwno.getCenter();

			if (flag)break;
			
				x = x + tr.x;
				y = y + tr.y;
				z = z + tr.z;
				metritis++;
			


		}
		if (flag == false) {
			cm_antikeimenwn[i].x = x / (float(metritis));
			cm_antikeimenwn[i].y = y / (float(metritis));
			cm_antikeimenwn[i].z = z / (float(metritis));
			x = 0;
			y = 0;
			z = 0;
		}

	}

	//edw enwnw ta cm gia na ftiaksw ton reeb graph//
	for (int i = 0; i < antikeimena_mesh.size()-1; i++) {
		int met = 0;
		for (int j = i; j < katigoria.size(); j++) {
			bool flag = false;
			if (katigoria[i] == katigoria[j] - 1) {
				LineSeg3D line = LineSeg3D(cm_antikeimenwn[i].x, cm_antikeimenwn[i].y, cm_antikeimenwn[i].z, cm_antikeimenwn[j].x, cm_antikeimenwn[j].y, cm_antikeimenwn[j].z, vvr::Colour::darkOrange);

				for (int k = 0; k < antikeimena_mesh[i].getTriangles().size(); k++) {
					vvr::Triangle trig = antikeimena_mesh[i].getTriangles()[k];
					vec p1 = trig.v1();
					vec p2 = trig.v2();
					vec p3 = trig.v3();

					if (FindAdjacentTriangle(antikeimena_mesh[j], p1, p2) || FindAdjacentTriangle(antikeimena_mesh[j], p3, p2) || FindAdjacentTriangle(antikeimena_mesh[j], p3, p1)) {
						met++;
						
					}
				}
				//gia na akoumpane kala oi katigories
				if (met >= 5 ) {//if (flag == true) {
					reeb_lines.push_back(line);
						
				}
				met = 0;
			}
		}
	}

	//ta xeria armadilo
	vector<vec> simeia_telika;
	int reeb_size = reeb_lines.size();
	for (int i = 0; i < reeb_size; i++) {
		LineSeg3D line = reeb_lines[i];

		vector<vec> point;
		point.push_back(vec(line.x1, line.y1, line.z1));
		point.push_back(vec(line.x2, line.y2, line.z2));

		bool flag = false;
		for (int k = 0; k < 2; k++) {
			int metritis = 0;
			for (int j = 0; j < reeb_size; j++) {
				LineSeg3D line2 = reeb_lines[j];
				vec start2;
				start2.x = line2.x1;
				start2.y = line2.y1;
				start2.z = line2.z1;

				vec end2;
				end2.x = line2.x2;
				end2.y = line2.y2;
				end2.z = line2.z2;
				if (j != i) {
					if (point[k].x == start2.x && point[k].y == start2.y && point[k].z == start2.z) { metritis++; }

					else 	if (point[k].x == end2.x && point[k].y == end2.y && point[k].z == end2.z) { metritis++; }
				}
			}
			if (metritis == 0) {
				simeia_telika.push_back(point[k]);
 
				}
			}
		}
	
	for (int i = 0; i < telika_akraia_si.size(); i++) {
		float min_ds = inf;
		int deik;
		for (int j = 0; j < simeia_telika.size(); j++) {
		

			if (simeia_telika[j].Distance(telika_akraia_si[i]) < min_ds) {
				min_ds = simeia_telika[j].Distance(telika_akraia_si[i]);
				deik = j;
			}

		}
	
		reeb_lines.push_back(LineSeg3D(simeia_telika[deik].x, simeia_telika[deik].y, simeia_telika[deik].z, telika_akraia_si[i].x, telika_akraia_si[i].y, telika_akraia_si[i].z, vvr::Colour::darkOrange));

	
	
	}

	//diagrafw apo ton reeb tis grammes pou uparxoun alla den kataligoun se akaria simeia
	for (int i = 0; i < reeb_lines.size(); i++) {
		LineSeg3D line = reeb_lines[i];

		vector<vec> point;
		point.push_back(vec(line.x1, line.y1, line.z1));
		point.push_back(vec(line.x2, line.y2, line.z2));
	
		bool flag = false;
		for (int k = 0; k < 2; k++) {
			int metritis = 0;
				for (int j = 0; j < reeb_lines.size(); j++) {
					LineSeg3D line2 = reeb_lines[j];
					vec start2;
					start2.x = line2.x1;
					start2.y = line2.y1;
					start2.z = line2.z1;

					vec end2;
					end2.x = line2.x2;
					end2.y = line2.y2;
					end2.z = line2.z2;
					if (j != i) {
						if (point[k].x == start2.x && point[k].y == start2.y && point[k].z == start2.z) { metritis++; }

						else 	if (point[k].x == end2.x && point[k].y == end2.y && point[k].z == end2.z) { metritis++; }


					}

				}
			if (metritis == 0) {
			for (int g = 0; g < telika_akraia_si.size(); g++) {

				if (telika_akraia_si[g].x == point[k].x && telika_akraia_si[g].y == point[k].y && telika_akraia_si[g].z == point[k].z) { flag = true; }

			}
			if (flag == false) {
				  reeb_lines.erase(reeb_lines.begin() + i);
				  i--;
				break;
			}
		}
		}
	}
 
}
void Mesh3DScene::telika_akria(int ideiktis) {
 
	
	GeodesicDistance();
	 
	 
	geodesiakes_points.resize(m_model.getVertices().size(), 0);
	float max = 0, min = inf;
	for (int i = 0; i < m_model.getVertices().size(); i++) {

		float mindist = inf;
		int metritis = 0;
		float geod_min;
		for (int j = 0; j < m_model.getTriangles().size(); j++) {
			vvr::Triangle trig = m_model.getTriangles()[j];
			if (trig.v[0] == i || trig.v[1]==i || trig.v[2]==i) {


				geodesiakes_points[i] = geodesiakes_points[i] + geodesiakes[j];
				if (mindist > trig.getCenter().Distance(m_model.getVertices()[i])) {
				
					mindist = trig.getCenter().Distance(m_model.getVertices()[i]);
					geod_min = geodesiakes[j];
				}
				metritis++;
				
			}
		}
		geodesiakes_points[i] = (geodesiakes_points[i]+5*geod_min)/( metritis+5);
		if (geodesiakes_points[i] > max)max = geodesiakes_points[i];
		if (geodesiakes_points[i] < min)min = geodesiakes_points[i];

		//echo(metritis);
	}

	 

	telika_akraia_simeia.resize(telika_akraia_simeia.size() + 1);

	for (int i = 0; i < m_model.getVertices().size(); i++) {
		bool flag = true;
		float orio = (max - min) / 20;
		//echo(orio);
		//echo(min);
		if (geodesiakes_points[i] < (orio*8 + min)) {
			flag = false;
		}
		vec point = m_model.getVertices()[i];
		
		for (int j = 0; j < m_model.getTriangles().size(); j++) {
			vvr::Triangle trig = m_model.getTriangles()[j];
			if (point.x == trig.v1().x && point.y == trig.v1().y && point.z == trig.v1().z) {
				if (geodesiakes_points[trig.vi2] * 1.001 > geodesiakes_points[i] || geodesiakes_points[trig.vi3] * 1.001 > geodesiakes_points[i]  ) {
					flag = false;
					break;
				}
				for (int k = 0; k < m_model.getTriangles().size(); k++) {
					vvr::Triangle trig2 = m_model.getTriangles()[k];
					if ((trig.vi2 == trig2.vi1 || trig.vi2 == trig2.vi2 || trig.vi2 == trig2.vi3) && (trig.vi3 == trig2.vi1 || trig.vi3 == trig2.vi2 || trig.vi3 == trig2.vi3)) {
						if (trig.vi1 != trig2.vi1 && trig.vi1 != trig2.vi2 && trig.vi1 != trig2.vi3) {
							if (geodesiakes_points[trig2.vi2] * 1.001 > geodesiakes_points[i]  || geodesiakes_points[trig2.vi3] * 1.001> geodesiakes_points[i] || geodesiakes_points[trig2.vi1] * 1.001> geodesiakes_points[i] ){//|| geodesiakes[k]>geodesiakes_points[i]) {
								flag = false;
								break;
							}
						}
					}
				}
			}

			if (point.x == trig.v2().x && point.y == trig.v2().y && point.z == trig.v2().z) {
				if (geodesiakes_points[trig.vi1] * 1.001 > geodesiakes_points[i] || geodesiakes_points[trig.vi3] * 1.001 > geodesiakes_points[i] ) {
					flag = false;
					break;
				}

				for (int k = 0; k < m_model.getTriangles().size(); k++) {
					vvr::Triangle trig2 = m_model.getTriangles()[k];
					if ((trig.vi1 == trig2.vi1 || trig.vi1 == trig2.vi2 || trig.vi1 == trig2.vi3) && (trig.vi3 == trig2.vi1 || trig.vi3 == trig2.vi2 || trig.vi3 == trig2.vi3)) {

						if (trig.vi2 != trig2.vi1 && trig.vi2 != trig2.vi2 && trig.vi2 != trig2.vi3) {
							if (geodesiakes_points[trig2.vi2] * 1.001 > geodesiakes_points[i] || geodesiakes_points[trig2.vi3] * 1.001 > geodesiakes_points[i] || geodesiakes_points[trig2.vi1] * 1.001 > geodesiakes_points[i]) {//|| geodesiakes[k]>geodesiakes_points[i]) {//) {
								flag = false;
								break;
							}

						}
					}

				}
			}

			if (point.x == trig.v3().x && point.y == trig.v3().y && point.z == trig.v3().z) {
				if (geodesiakes_points[trig.vi2] * 1.001 > geodesiakes_points[i] || geodesiakes_points[trig.vi1] * 1.001 > geodesiakes_points[i]  ) {
					flag = false;
					break;
				}

				for (int k = 0; k < m_model.getTriangles().size(); k++) {
					vvr::Triangle trig2 = m_model.getTriangles()[k];
					if ((trig.vi2 == trig2.vi1 || trig.vi2 == trig2.vi2 || trig.vi2 == trig2.vi3) && (trig.vi1 == trig2.vi1 || trig.vi1 == trig2.vi2 || trig.vi1 == trig2.vi3)) {

						if (trig.vi3 != trig2.vi1 && trig.vi3 != trig2.vi2 && trig.vi3 != trig2.vi3) {
							if (geodesiakes_points[trig2.vi2] * 1.001 > geodesiakes_points[i] || geodesiakes_points[trig2.vi3] * 1.001 > geodesiakes_points[i] || geodesiakes_points[trig2.vi1] * 1.001> geodesiakes_points[i]) {//|| geodesiakes[k]>geodesiakes_points[i]) {
								flag = false;
								break;
							}
						}
					}
				}
			}
		}

			if (flag == true) {
				telika_akraia_si.push_back(point);
			}
	}
	 
}
void Mesh3DScene::antikeimena(std::vector<std::pair<int, int>> intexes, std::vector<int> &processed,int thesi,int metritis_antikeimenwn)
{

	if (processed[thesi] == 1)return;
	processed[thesi] = 1;

	vvr::Triangle trigwno = m_model.getTriangles()[intexes[thesi].first];
	if (intexes[thesi].second == 2) {
		trigwno = greenMesh.getTriangles()[intexes[thesi].first];
	}
	vec a = trigwno.v1();
	vec b = trigwno.v2();
	vec c = trigwno.v3();

	antikeimena_mesh[metritis_antikeimenwn].getVertices().push_back(a);
	antikeimena_mesh[metritis_antikeimenwn].getVertices().push_back(b);
	antikeimena_mesh[metritis_antikeimenwn].getVertices().push_back(c);
	int index = antikeimena_mesh[metritis_antikeimenwn].getVertices().size() - 3;
	antikeimena_mesh[metritis_antikeimenwn].getTriangles().push_back(vvr::Triangle(&antikeimena_mesh[metritis_antikeimenwn].getVertices(), index, index + 1, index + 2));

	if(metritis_antikeimenwn==0)
	{
		redMesh.getVertices().push_back(a);
		redMesh.getVertices().push_back(b);
		redMesh.getVertices().push_back(c);
		index = redMesh.getVertices().size() - 3;
		redMesh.getTriangles().push_back(vvr::Triangle(&redMesh.getVertices(), index, index + 1, index + 2)); 
	}

	int thesi_prwtou=-1;
	if (FindAdjacentTriangle(intexes, a, b, thesi_prwtou,thesi)) {
			antikeimena(intexes, processed, thesi_prwtou,  metritis_antikeimenwn);
	}
	int thesi_deutrerou = -1;
	if (FindAdjacentTriangle(intexes, b, c, thesi_deutrerou,thesi)) {
			antikeimena(intexes, processed, thesi_deutrerou,metritis_antikeimenwn);
	}
	int thesi_tritou = -1;
	if (FindAdjacentTriangle(intexes, a, c, thesi_tritou,thesi)) {
			antikeimena(intexes, processed, thesi_tritou, metritis_antikeimenwn);
	}

}
bool Mesh3DScene::FindAdjacentTriangle(std::vector<std::pair<int, int>> intexes , vec p1, vec p2, int & tri_adj_index,int thesi)
{
	 

	for (int i = 0; i < intexes.size(); i++) {
		if (i != thesi) {
			vvr::Triangle trigwno = m_model.getTriangles()[intexes[i].first];




			if (intexes[i].second == 2) {
				trigwno = greenMesh.getTriangles()[intexes[i].first];
			}
			vec a = trigwno.v1();
			vec b = trigwno.v2();
			vec c = trigwno.v3();

			if ((abs(a.x-p1.x)<0.001 && abs(a.y - p1.y)<0.001 && abs(a.z - p1.z)<0.001 && abs(b.x - p2.x)<0.001 && abs(b.y - p2.y)<0.001 && abs(b.z - p2.z)<0.001) || (abs(b.x - p1.x)<0.001 && abs(b.y - p1.y)<0.001 && abs(b.z - p1.z)<0.001 &&abs(a.x - p2.x)<0.001 && abs(a.y - p2.y)<0.001 && abs(a.z - p2.z)<0.001)) {

				tri_adj_index = i;
				return true;
			}
			else if ((abs(a.x - p1.x)<0.001 && abs(a.y - p1.y)<0.001 && abs(a.z - p1.z)<0.001 && abs(c.x - p2.x)<0.001 && abs(c.y - p2.y)<0.001 && abs(c.z - p2.z)<0.001) || (abs(c.x - p1.x)<0.001 && abs(c.y - p1.y)<0.001 && abs(c.z - p1.z)<0.001 &&abs(a.x - p2.x)<0.001 && abs(a.y - p2.y)<0.001 && abs(a.z - p2.z)<0.001)) {

				tri_adj_index = i;
				return true;
			}
			else if ((abs(c.x - p1.x)<0.001 && abs(c.y - p1.y)<0.001 && abs(c.z - p1.z)<0.001 && abs(b.x - p2.x)<0.001 && abs(b.y - p2.y)<0.001 && abs(b.z - p2.z)<0.001) || (abs(b.x - p1.x)<0.001 && abs(b.y - p1.y)<0.001 && abs(b.z - p1.z)<0.001 &&abs(c.x - p2.x)<0.001 && abs(c.y - p2.y)<0.001 && abs(c.z - p2.z)<0.001)) {

				tri_adj_index = i;
				return true;
			}


 
		}
	}
	return false;
}
bool Line_Plane_Intersection(vec& contact, LineSeg3D line, Plane &plane) {
	vec dieuthinsi;
	dieuthinsi.x = (line.x2 - line.x1);
	dieuthinsi.y = (line.y2 - line.y1);
	dieuthinsi.z = (line.z2 - line.z1);
	vec start;
	start.x = line.x1;
	start.y = line.y1;
	start.z = line.z1;
	if (Dot(plane.normal, dieuthinsi) == 0) {
		return false;
	}
	float x = (plane.d - Dot(plane.normal, start)) / Dot(plane.normal, dieuthinsi);
	contact.x = start.x + dieuthinsi.x*x;
	contact.y = start.y + dieuthinsi.y*x;
	contact.z = start.z + dieuthinsi.z*x;
	return true;
}
bool Mesh3DScene::FindAdjacentTriangle(vvr::Mesh mesh,  vec p1, vec p2)
{ 

	for (int i = 0; i < mesh.getTriangles().size(); i++) {
	
			vvr::Triangle trigwno = mesh.getTriangles()[i];


			vec a = trigwno.v1();
			vec b = trigwno.v2();
			vec c = trigwno.v3();

			if ((abs(a.x - p1.x)<0.0001 && abs(a.y - p1.y)<0.000001 && abs(a.z - p1.z)<0.000001 && abs(b.x - p2.x)<0.000001 && abs(b.y - p2.y)<0.000001 && abs(b.z - p2.z)<0.000001) || (abs(b.x - p1.x)<0.000001 && abs(b.y - p1.y)<0.000001 && abs(b.z - p1.z)<0.000001 &&abs(a.x - p2.x)<0.000001 && abs(a.y - p2.y)<0.000001 && abs(a.z - p2.z)<0.000001)) {
				 
				return true;
			}
			else if ((abs(a.x - p1.x)<0.000001 && abs(a.y - p1.y)<0.000001 && abs(a.z - p1.z)<0.000001 && abs(c.x - p2.x)<0.000001 && abs(c.y - p2.y)<0.000001 && abs(c.z - p2.z)<0.000001) || (abs(c.x - p1.x)<0.000001 && abs(c.y - p1.y)<0.000001 && abs(c.z - p1.z)<0.000001 &&abs(a.x - p2.x)<0.000001 && abs(a.y - p2.y)<0.000001 && abs(a.z - p2.z)<0.000001)) {

		 
				return true;
			}
			else if ((abs(c.x - p1.x)<0.000001 && abs(c.y - p1.y)<0.000001 && abs(c.z - p1.z)<0.000001 && abs(b.x - p2.x)<0.000001 && abs(b.y - p2.y)<0.000001 && abs(b.z - p2.z)<0.000001) || (abs(b.x - p1.x)<0.000001 && abs(b.y - p1.y)<0.000001 && abs(b.z - p1.z)<0.000001 &&abs(c.x - p2.x)<0.000001 && abs(c.y - p2.y)<0.000001 && abs(c.z - p2.z)<0.000001)) {

			 
				return true;
			}


		}
	


	return false;

}
 void Mesh3DScene::partition() {

	reeb_graph_geodesic_distances();
	vector<float> antikeimena_curnature;
	antikeimena_curnature.resize(antikeimena_mesh.size(), float(0));
/* 
	for (int i = 0; i < antikeimena_mesh.size(); i++) {
		for (int j = 0; j < antikeimena_mesh[i].getVertices().size(); j++) {
			vec ant = antikeimena_mesh[i].getVertices()[j];
			for (int k = 0; k < m_model.getVertices().size(); k++) {
				vec mod = m_model.getVertices()[k];
				if (ant.x == mod.x && ant.y == mod.y && ant.z == mod.z) {
					//antikeimena_curnature[i] = antikeimena_curnature[i] + curvuture_vaues[k];
				}
			}
		}
		//antikeimena_curnature[i] = antikeimena_curnature[i] / antikeimena_mesh[i].getVertices().size();
	}
*/
	//diagrafi diplw grammwn reeb
	for (int i = 0; i < reeb_lines.size(); i++) {
		LineSeg3D line1 = reeb_lines[i];
		for (int j = 0; j < reeb_lines.size(); j++) {
			LineSeg3D line2 = reeb_lines[j];
			if (i != j) {
				if ((line1.x1 == line2.x1 && line1.y1 == line2.y1 && line1.z1 == line2.z1  && line1.x2 == line2.x2 && line1.y2 == line2.y2 && line1.z2 == line2.z2) || (line1.x1 == line2.x2 && line1.y1 == line2.y2 && line1.z1 == line2.z2  && line1.x2 == line2.x1 && line1.y2 == line2.y1 && line1.z2 == line2.z1)) {
					reeb_lines[j].x1 = inf;
					reeb_lines[j].y1 = inf;
					reeb_lines[j].z1 = inf;
					reeb_lines[j].x2 = inf;
					reeb_lines[j].y2 = inf;
					reeb_lines[j].z2 = inf;
				}
			}
		}
	}
	for (int i = 0; i < reeb_lines.size(); i++)
	{
		if (reeb_lines[i].x1 == inf && reeb_lines[i].y1 == inf && reeb_lines[i].z1 == inf && reeb_lines[i].x2 == inf && reeb_lines[i].y2 == inf && reeb_lines[i].z2 == inf) {
			reeb_lines.erase(reeb_lines.begin() + i);
			i--;
		}
	}
	/////simeia reeb_graph
	vector<vec> reeb_point;
	for (int i = 0; i < reeb_lines.size(); i++) {

		LineSeg3D line2 = reeb_lines[i];
		vec start2, end2;
		start2.x = line2.x1;
		start2.y = line2.y1;
		start2.z = line2.z1;
		end2.x = line2.x2;
		end2.y = line2.y2;
		end2.z = line2.z2;
		bool f1 = true, f2 = true;
		for (int j = 0; j < reeb_point.size(); j++) {

			if (start2.x == reeb_point[j].x && start2.y == reeb_point[j].y && start2.z == reeb_point[j].z) { f1 = false; }
			if (end2.x == reeb_point[j].x && end2.y == reeb_point[j].y && end2.z == reeb_point[j].z) { f2 = false; }
		}
		if (f1)reeb_point.push_back(start2);
		if (f2)reeb_point.push_back(end2);
		f1 = true; f2 = true;
	}


////junction points
	vector<vec> junction_points;
	//kati.resize(reeb_lines.size());
	//echo(reeb_lines.size());
	for (int i = 0; i < reeb_lines.size(); i++) {
		LineSeg3D line = reeb_lines[i];
		vec start, end;
		start.x = line.x1;
		start.y = line.y1;
		start.z = line.z1;
		end.x = line.x2;
		end.y = line.y2;
		end.z = line.z2;
		
			int met1 = 0,met2=0;
			for (int j = 0; j < reeb_lines.size(); j++) {
				if (i != j) {
					LineSeg3D line2 = reeb_lines[j];
					vec start2, end2;
					start2.x = line2.x1;
					start2.y = line2.y1;
					start2.z = line2.z1;
					end2.x = line2.x2;
					end2.y = line2.y2;
					end2.z = line2.z2;
					if ((start.x == start2.x && start.y == start2.y && start.z == start2.z) || (start.x == end2.x && start.y == end2.y && start.z == end2.z)) { met1++; }
					if ((end.x == start2.x && end.y == start2.y && end.z == start2.z) || (end.x == end2.x && end.y == end2.y && end.z == end2.z)) { met2++; }


				}
			}
			if (met1 >=2) {// kati[i] = 2; 
			pointset3.push_back(start); junction_points.push_back(start);}
			if(met2 >= 2) { //kati[i] = 2;
			pointset3.push_back(end); junction_points.push_back(end);}
		

			met1 = 0; met2 = 0;
		}
	////////////////////
	//////////////////////////////meloi---apo akraia se junction
	int metritis = 0;
	bool flag = false;
	//meloi.resize(meloi.size() + 1);
	kati.resize(reeb_lines.size(), 1);
	for (int j = 0; j < telika_akraia_si.size(); j++) {
		vec start_p = telika_akraia_si[j];
		vec proig = vec(inf, inf, inf);
		if (flag == false)	meloi.resize(meloi.size() + 1);
		flag = true;
		for (int i = 0; i < reeb_lines.size(); i++) {
			if (kati[i] == 1) {
				LineSeg3D line = reeb_lines[i];
				vec start, end;
				start.x = line.x1;
				start.y = line.y1;
				start.z = line.z1;
				end.x = line.x2;
				end.y = line.y2;
				end.z = line.z2;
				if ((start.x == start_p.x && start.y == start_p.y && start.z == start_p.z) || (end.x == start_p.x && end.y == start_p.y && end.z == start_p.z) && ((proig.x != start_p.x && proig.y != start_p.y && proig.z != start_p.z))) {
					vec end_p;
					if (start.x == start_p.x && start.y == start_p.y && start.z == start_p.z)end_p = end;
					if (end.x == start_p.x && end.y == start_p.y && end.z == start_p.z) end_p = start;


					    meloi[metritis].push_back(line);
					kati[i] = 3;

					i = 0;
					proig = start_p;
					start_p = end_p;
					flag = false;
					for (int l = 0; l < junction_points.size(); l++) {
					
						if (start_p.x == junction_points[l].x && start_p.y == junction_points[l].y && start_p.z == junction_points[l].z) { i = 2000; break; }
					
					}
					
				}

			}
		}

		 if (flag == false)	metritis++;
	}
	/////////////////meloi apo junction se junction
	 flag = false;
	//meloi.resize(meloi.size() + 1);
	//kati.resize(reeb_lines.size(), 1);
	for (int j = 0; j < junction_points.size(); j++) {
		vec start_p = junction_points[j];
		vec proig = vec(inf, inf, inf);
		if (flag == false)	meloi.resize(meloi.size() + 1);
		flag = true;
		for (int i = 0; i < reeb_lines.size(); i++) {
			if (kati[i] == 1) {
				LineSeg3D line = reeb_lines[i];
				vec start, end;
				start.x = line.x1;
				start.y = line.y1;
				start.z = line.z1;
				end.x = line.x2;
				end.y = line.y2;
				end.z = line.z2;
				if ((start.x == start_p.x && start.y == start_p.y && start.z == start_p.z) || (end.x == start_p.x && end.y == start_p.y && end.z == start_p.z) && ((proig.x != start_p.x && proig.y != start_p.y && proig.z != start_p.z))) {
					vec end_p;
					if (start.x == start_p.x && start.y == start_p.y && start.z == start_p.z)end_p = end;
					if (end.x == start_p.x && end.y == start_p.y && end.z == start_p.z) end_p = start;
					 meloi[metritis].push_back(line);
					kati[i] = 3;

					i = -1;
					proig = start_p;
					start_p = end_p;
					flag = false;
					for (int l = 0; l < junction_points.size(); l++) {

						if (start_p.x == junction_points[l].x && start_p.y == junction_points[l].y && start_p.z == junction_points[l].z) { i = 2000; break; }

					}

				}

			}
		}

		if (flag == false)	metritis++;
	}
	////////////////////antistoixeia
	for (int i = 0; i < m_model.getTriangles().size(); i++) {
		vvr::Triangle trig = m_model.getTriangles()[i];
		vec point = trig.getCenter();
		float dist = inf;
		vec deik;
		for (int j = 0; j < reeb_point.size(); j++) {
			bool flag = true;
			for (int r = 0; r < telika_akraia_si.size(); r++) {
				if (reeb_point[j].x == telika_akraia_si[r].x  && reeb_point[j].y == telika_akraia_si[r].y && reeb_point[j].z == telika_akraia_si[r].z) { flag = false; break; }

			}
			for (int r = 0; r < junction_points.size(); r++) {
				if (reeb_point[j].x == junction_points[r].x  && reeb_point[j].y == junction_points[r].y && reeb_point[j].z == junction_points[r].z) { flag = false; break; }

			}
			if (flag == true){

				if (dist > reeb_point[j].Distance(point)) {
					dist = reeb_point[j].Distance(point);
					deik = reeb_point[j];
				}}}

		antistoixeia.push_back(make_pair(deik, dist));

	}
	for (int j = 0; j < reeb_point.size(); j++) {
		for (int i = 0; i < cm_antikeimenwn.size(); i++) {
			if (reeb_point[j].x == cm_antikeimenwn[i].x  && reeb_point[j].y == cm_antikeimenwn[i].y && reeb_point[j].z == cm_antikeimenwn[i].z) {
				for (int k = 0; k < antikeimena_mesh[i].getTriangles().size(); k++) {
					vec ant = antikeimena_mesh[i].getTriangles()[k].getCenter();
					for (int l = 0; l < m_model.getTriangles().size(); l++) {
						vec mod = m_model.getTriangles()[l].getCenter();
						if (ant.x == mod.x && ant.y == mod.y && ant.z == mod.z) {
							if (antistoixeia[l].second>2*cm_antikeimenwn[i].Distance(mod)) {
								 //   antistoixeia[l].first = cm_antikeimenwn[i];
								  //  antistoixeia[l].second = cm_antikeimenwn[i].Distance(mod);
							}
						}
					}
				}
			}
		}
	}
	
	 
	trigwna_antikeimenwn.resize(meloi.size());
	for (int i = 0; i < meloi.size(); i++) {
		 vector< vec> simeia_melous;
		simeia_komatiou(meloi[i], simeia_melous);
		for (int j = 0; j < simeia_melous.size(); j++) {
			for (int k = 0; k < m_model.getTriangles().size(); k++) {
				
				if (simeia_melous[j].x == antistoixeia[k].first.x  && simeia_melous[j].y == antistoixeia[k].first.y && simeia_melous[j].z == antistoixeia[k].first.z) {
					trigwna_antikeimenwn[i].push_back(m_model.getTriangles()[k]);
				
				}
			}
		}
	}

	//	echo(meloi.size());
	//	echo(trigwna_antikeimenwn.size());

	  
} 
void Mesh3DScene::motion() {

	reeb_graph_geodesic_distances();
	//diagrafi diplw grammwn reeb
	for (int i = 0; i < reeb_lines.size(); i++) {
		LineSeg3D line1 = reeb_lines[i];
		for (int j = 0; j < reeb_lines.size(); j++) {
			LineSeg3D line2 = reeb_lines[j];
			if (i != j) {
				if ((line1.x1 == line2.x1 && line1.y1 == line2.y1 && line1.z1 == line2.z1  && line1.x2 == line2.x2 && line1.y2 == line2.y2 && line1.z2 == line2.z2) || (line1.x1 == line2.x2 && line1.y1 == line2.y2 && line1.z1 == line2.z2  && line1.x2 == line2.x1 && line1.y2 == line2.y1 && line1.z2 == line2.z1)) {
					reeb_lines[j].x1 = inf;
					reeb_lines[j].y1 = inf;
					reeb_lines[j].z1 = inf;
					reeb_lines[j].x2 = inf;
					reeb_lines[j].y2 = inf;
					reeb_lines[j].z2 = inf;
				}
			}
		}
	}
	for (int i = 0; i < reeb_lines.size(); i++)
	{
		if (reeb_lines[i].x1 == inf && reeb_lines[i].y1 == inf && reeb_lines[i].z1 == inf && reeb_lines[i].x2 == inf && reeb_lines[i].y2 == inf && reeb_lines[i].z2 == inf) {
 
			reeb_lines.erase(reeb_lines.begin() + i);
			i--;
		}
	}
	//posa afinei aristera kai deksia
	for (int i = 0; i < reeb_lines.size(); i++) {
		LineSeg3D line2 = reeb_lines[i];
		vec start2;
		start2.x = line2.x1;
		start2.y = line2.y1;
		start2.z = line2.z1;
		vec cent2;
		cent2.x = 0.5*(line2.x1 + line2.x2);
		cent2.y = 0.5*(line2.y1 + line2.y2);
		cent2.z = 0.5*(line2.z1 + line2.z2);
		vec end2;
		end2.x = line2.x2;
		end2.y = line2.y2;
		end2.z = line2.z2;
		Plane plane = Plane(cent2, vec(cent2.x, cent2.y - 2, cent2.z), vec(cent2.x - 2, cent2.y, cent2.z));
		plane_t = Plane(cent2, vec(cent2.x, cent2.y - 2, cent2.z), vec(cent2.x - 2, cent2.y, cent2.z));
		int metr1 = 0, metr2 = 0;
		for (int j = 0; j < reeb_lines.size(); j++) {
			if (i != j) {
				LineSeg3D line1 = reeb_lines[j];
				vec cent;
				cent.x = 0.5*(line1.x1 + line1.x2);
				cent.y = 0.5*(line1.y1 + line1.y2);
				cent.z = 0.5*(line1.z1 + line1.z2);
				vec start;
				start.x = line1.x1;
				start.y = line1.y1;
				start.z = line1.z1;

				if ((Dot(plane.normal, cent) - plane.d) > 0) {
					metr1++;
				}
				else { metr2++; }

			}
		}
		aris_deks.push_back(make_pair(metr1, metr2)); //echo(metr1); echo(metr2);
		metr1 = 0; metr2 = 0;
	}

	int  min_diaf = abs(200000);
	int dekt_min_dif;
	for (int i = 0; i < aris_deks.size(); i++) {
		if( abs(aris_deks[i].first-aris_deks[i].second)<min_diaf){
			min_diaf = abs(aris_deks[i].first - aris_deks[i].second);
			dekt_min_dif = i;
		}

	}
	//////////////
	vector<vec> junction_points;
	//kati.resize(reeb_lines.size());
	//echo(reeb_lines.size());
	for (int i = 0; i < reeb_lines.size(); i++) {
		LineSeg3D line = reeb_lines[i];
		vec start, end;
		start.x = line.x1;
		start.y = line.y1;
		start.z = line.z1;
		end.x = line.x2;
		end.y = line.y2;
		end.z = line.z2;

		int met1 = 0, met2 = 0;
		for (int j = 0; j < reeb_lines.size(); j++) {
			if (i != j) {
				LineSeg3D line2 = reeb_lines[j];
				vec start2, end2;
				start2.x = line2.x1;
				start2.y = line2.y1;
				start2.z = line2.z1;
				end2.x = line2.x2;
				end2.y = line2.y2;
				end2.z = line2.z2;
				if ((start.x == start2.x && start.y == start2.y && start.z == start2.z) || (start.x == end2.x && start.y == end2.y && start.z == end2.z)) { met1++; }
				if ((end.x == start2.x && end.y == start2.y && end.z == start2.z) || (end.x == end2.x && end.y == end2.y && end.z == end2.z)) { met2++; }


			}
		}
		if (met1 >= 2) { 
			  junction_points.push_back(start);
		}
		if (met2 >= 2) { 
			  junction_points.push_back(end);
		}


		met1 = 0; met2 = 0;
	}
	///////////////////////diagrafi diplwn junction point
	for (int i = 0; i < junction_points.size(); i++) {
		vec point1 = junction_points[i];
		for (int j = 0; j < junction_points.size(); j++) {
			vec point2 = junction_points[j];
			if (i != j) {
				if (point1.x == point2.x && point1.y == point2.y && point1.z == point2.z ) {
					junction_points[j].x = 228360;
					junction_points[j].y = 228360;
					junction_points[j].z = 228360;
					 
				}
			}
		}
	}
	for (int i = 0; i < junction_points.size(); i++)
	{
		if (junction_points[i].x  == 228360 && junction_points[i].y  == 228360 && junction_points[i].z  == 228360) {

			junction_points.erase(junction_points.begin() + i);
			i--;
		}
	}
	//////////////////////////////////////////////////////
	 ////aksonas
	aksonas.push_back(reeb_lines[dekt_min_dif]);
	vec starting;

	vec ending;

	vector<int> visitied;
	visitied.resize(reeb_lines.size(),0);
	vec proig;

	for (int u = 0; u < 2; u++) {
		if (u == 0) {
		
			starting.x = aksonas[0].x1;
			starting.y = aksonas[0].y1;
			starting.z = aksonas[0].z1;

			ending.x = aksonas[0].x2;
			ending.y = aksonas[0].y2;
			ending.z = aksonas[0].z2;
			
			visitied[dekt_min_dif] = 1;

			vec proig = ending;
		
		}if (u == 1) {
		
			starting.x = aksonas[0].x2;
			starting.y = aksonas[0].y2;
			starting.z = aksonas[0].z2;

			ending.x = aksonas[0].x1;
			ending.y = aksonas[0].y1;
			ending.z = aksonas[0].z1;
			
			vec proig = ending;

 
		}
 
		for (int i = 0; i < reeb_lines.size(); i++) {
			if (visitied[i] == 0)
			{
				LineSeg3D line2 = reeb_lines[i];
				vec start2, end2;
				start2.x = line2.x1;
				start2.y = line2.y1;
				start2.z = line2.z1;
				end2.x = line2.x2;
				end2.y = line2.y2;
				end2.z = line2.z2;
				if (((starting.x == start2.x && starting.y == start2.y && starting.z == start2.z) || (starting.x == end2.x && starting.y == end2.y && starting.z == end2.z)) && ((proig.x != start2.x && proig.y != start2.y && proig.z != start2.z) && (proig.x != end2.x && proig.y != end2.y && proig.z != end2.z))) {
					int temp_min_dek;
					bool flag = true;
					for (int t = 0; t < junction_points.size(); t++) {

						if (junction_points[t].x == starting.x && junction_points[t].y == starting.y && junction_points[t].z == starting.z) {

							int temp_mi = 20000;


							for (int k = 0; k < reeb_lines.size(); k++) {
								if (visitied[i] == 0) {
									LineSeg3D line3 = reeb_lines[k];
									vec start3, end3;
									start3.x = line3.x1;
									start3.y = line3.y1;
									start3.z = line3.z1;
									end3.x = line3.x2;
									end3.y = line3.y2;
									end3.z = line3.z2;
									if ((junction_points[t].x == start3.x && junction_points[t].y == start3.y && junction_points[t].z == start3.z) || (junction_points[t].x == end3.x && junction_points[t].y == end3.y && junction_points[t].z == end3.z) && ((proig.x != start3.x && proig.y != start3.y && proig.z != start3.z) && (proig.x != end3.x && proig.y != end3.y && proig.z != end3.z))) {

										if (abs(start3.y - end3.y) < temp_mi) {
											temp_mi = abs(start3.y - end3.y);
											temp_min_dek = k;
											flag = false;
										}

									}

								}
							}

							break;
						}
					}
					if (flag) {
						aksonas.push_back(reeb_lines[i]);
						visitied[i] = 1;
						proig = starting;
						if (starting.x == start2.x && starting.y == start2.y && starting.z == start2.z) { starting = end2; }
						else if (starting.x == end2.x && starting.y == end2.y && starting.z == end2.z) { starting = start2; }

						i = -1;
					}
					else {
						LineSeg3D line3 = reeb_lines[temp_min_dek];
						vec start3, end3;
						start3.x = line3.x1;
						start3.y = line3.y1;
						start3.z = line3.z1;
						end3.x = line3.x2;
						end3.y = line3.y2;
						end3.z = line3.z2;
						aksonas.push_back(reeb_lines[temp_min_dek]);
						visitied[temp_min_dek] = 1;
						proig = starting;
						if (starting.x == start3.x && starting.y == start3.y && starting.z == start3.z) { starting = end3; }
						else if (starting.x == end3.x && starting.y == end3.y && starting.z == end3.z) { starting = start3; }
						i = -1;
					}
				}
			}

		}

	}
	//////////////////////////ama exei stil xeri to bgazw

	std::vector<math::vec> aksonas_point;
	simeia_komatiou(aksonas, aksonas_point);
	int met = 0;
	for (int i = 0; i < junction_points.size(); i++) {
		int met = 0;
		for (int j = 0; j < aksonas_point.size(); j++) {
			if (junction_points[i].x == aksonas_point[j].x && junction_points[i].y == aksonas_point[j].y && junction_points[i].z == aksonas_point[j].z) { met++; }

		}
		if (met == 0) {
			int prwti_fora = 0;
			for (int k = 0; k < reeb_lines.size(); k++) {
				LineSeg3D line2 = reeb_lines[k];
				vec start2, end2;
				start2.x = line2.x1;
				start2.y = line2.y1;
				start2.z = line2.z1;
				end2.x = line2.x2;
				end2.y = line2.y2;
				end2.z = line2.z2;
				if ((start2.x == junction_points[i].x && start2.y == junction_points[i].y && start2.z == junction_points[i].z) || (end2.x == junction_points[i].x && end2.y == junction_points[i].y && end2.z == junction_points[i].z)) {

					for (int g = 0; g < telika_akraia_si.size(); g++) {

						if ((start2.x == telika_akraia_si[g].x && start2.y == telika_akraia_si[g].y && start2.z == telika_akraia_si[g].z) || (end2.x == telika_akraia_si[g].x && end2.y == telika_akraia_si[g].y && end2.z == telika_akraia_si[g].z)) {
							if (prwti_fora != 0) {
								reeb_lines.erase(reeb_lines.begin() + k);
								k--;

							}

							prwti_fora++;

						}
					}
				}
			}
		}
	}





	////oles oi grammes pou enontnai me junction point
	kati.resize(reeb_lines.size());
	//echo(reeb_lines.size());
	for (int i = 0; i < reeb_lines.size(); i++) {
		LineSeg3D line = reeb_lines[i];
		vec start, end;
		start.x = line.x1;
		start.y = line.y1;
		start.z = line.z1;
		end.x = line.x2;
		end.y = line.y2;
		end.z = line.z2;

		int met1 = 0, met2 = 0;
		for (int j = 0; j < reeb_lines.size(); j++) {
			if (i != j) {
				LineSeg3D line2 = reeb_lines[j];
				vec start2, end2;
				start2.x = line2.x1;
				start2.y = line2.y1;
				start2.z = line2.z1;
				end2.x = line2.x2;
				end2.y = line2.y2;
				end2.z = line2.z2;
				if ((start.x == start2.x && start.y == start2.y && start.z == start2.z) || (start.x == end2.x && start.y == end2.y && start.z == end2.z)) { met1++; }
				if ((end.x == start2.x && end.y == start2.y && end.z == start2.z) || (end.x == end2.x && end.y == end2.y && end.z == end2.z)) { met2++; }
			}
		}
		if (met1 >= 2 || met2 >= 2) { kati[i] = 2; }
		else { kati[i] = 1; }

		met1 = 0; met2 = 0;
	}

	////////////////////junction point with smallest x
	vec junct_min_x=vec(inf,inf,inf);
	for (int i = 0; i < junction_points.size(); i++) {
		if (junction_points[i].x < junct_min_x.x) {
			junct_min_x = junction_points[i];
		}
	}
	for (int i = 0; i < aksonas.size(); i++) {
		
		LineSeg3D line = aksonas[i];
		vec start, end;
		start.x = line.x1;
		start.y = line.y1;
		start.z = line.z1;
		end.x = line.x2;
		end.y = line.y2;
		end.z = line.z2;
		if (start.x < junct_min_x.x && end.x < junct_min_x.x) {
			oura.push_back(line);
		}
	
	
	
	}


 
	// markarw oses grammes einai ston aksona opote 8a mou minoun mono ta poddia
		for (int j = 0; j < reeb_lines.size(); j++) {
			LineSeg3D line = reeb_lines[j];
			vec start, end;
			start.x = line.x1;
			start.y = line.y1;
			start.z = line.z1;
			end.x = line.x2;
			end.y = line.y2;
			end.z = line.z2;
			for (int i = 0; i < aksonas.size(); i++) {
				LineSeg3D line2 = aksonas[i];
				vec start2, end2;
				start2.x = line2.x1;
				start2.y = line2.y1;
				start2.z = line2.z1;
				end2.x = line2.x2;
				end2.y = line2.y2;
				end2.z = line2.z2;
				if ((start.x == start2.x && start.y == start2.y && start.z == start2.z) && (end.x == end2.x && end.y == end2.y && end.z == end2.z) || ((start.x == end2.x && start.y == end2.y && start.z == end2.z) && (end.x == start2.x && end.y == start2.y && end.z == start2.z))) {
					kati[j] = 2;
				}


		}
	
	
	}
 
	///////////////////////////////////////meloi
	//oti den anoikei ston kurio aksona einai ta 4a podia gia ta opoia exoyme 2 mprosta kai duo piso i oura einai meta to teleutaio junction point omoiiso kai to kefali(an den eixame to kerato)
	//mporei omws na exoumme autia opote mporoume na poume meta to junction point pou arxizei ne exei klisi

	int metritis = 0;
	bool flag = false;
	//meloi.resize(meloi.size() + 1);
	for (int j = 0; j < telika_akraia_si.size(); j++) {
		vec start_p = telika_akraia_si[j];
		vec proig = vec(inf, inf, inf);
		if (flag == false)	meloi.resize(meloi.size() + 1);
		 flag = true;
		for (int i = 0; i < reeb_lines.size(); i++) {
			if (kati[i] == 1){
			LineSeg3D line = reeb_lines[i];
			vec start, end;
			start.x = line.x1;
			start.y = line.y1;
			start.z = line.z1;
			end.x = line.x2;
			end.y = line.y2;
			end.z = line.z2;
			if ((start.x == start_p.x && start.y == start_p.y && start.z == start_p.z) || (end.x == start_p.x && end.y == start_p.y && end.z == start_p.z) && ((proig.x != start_p.x && proig.y != start_p.y && proig.z != start_p.z))) {
				vec end_p;
				if (start.x == start_p.x && start.y == start_p.y && start.z == start_p.z)end_p = end;
				if (end.x == start_p.x && end.y == start_p.y && end.z == start_p.z) end_p = start;

				meloi[metritis].push_back(line);
				kati[i] = 3;

				i = -1;  
				proig = start_p;
				start_p = end_p;
				flag = false;
			}
		}
		}
	
		if(flag==false)	metritis++;
	}
 
	 //cm of meloi 
	float x = 0, y = 0, z = 0;
	int metri = 0;
	for (int i = 0; i < meloi.size(); i++) {
		for (int j = 0; j < meloi[i].size(); j++) {
			LineSeg3D line1 = meloi[i][j];
			vec cent;

			cent.x = 0.5*(line1.x1 + line1.x2);
			cent.y = 0.5*(line1.y1 + line1.y2);
			cent.z = 0.5*(line1.z1 + line1.z2);
			x = x + cent.x;
			y = y + cent.y;
			z = z + cent.z;
			metri++;
		}
	}
	vec kentro;
	kentro.x = x / float(metri);
	kentro.y = y / float(metri);
	kentro.z = z / float(metri);
	
	int dek=-1 , dek_m_d=-1 , dek_p_d=-1, dek_p_a;
	 

	//kai twra briskw se pio tetartimoria einai gia na dw pio podi einai apo ta tesera 
	flag = true;
	for (int i = 0; i <  meloi.size(); i++) {
		int metr_a = 0;
		for (int j = 0; j < meloi[i].size(); j++) {
		
			LineSeg3D line1 = meloi[i][j];
			vec cent;
			cent.x = 0.5*(line1.x1 + line1.x2);
			cent.y = 0.5*(line1.y1 + line1.y2);
			cent.z = 0.5*(line1.z1 + line1.z2);
		
			vec test;
			test.x = cent.x - kentro.x;
			test.y = cent.y - kentro.y;
			test.z = cent.z - kentro.z;

			if (test.x> 0 && test.z>0 && test.y<0) {//if ((Dot(plane1.normal,cent)-plane1.d)>0 && (Dot(plane2.normal, cent) - plane2.d)>0) {
				dek = i;
				flag = false; break;
			}
			else if (test.x > 0 && test.z < 0 && test.y<0) {
				dek_m_d = i; break;
			}
			else if (test.x < 0 && test.z > 0 && test.y<0) {
				dek_p_d = i; break;
			}
			else if (test.x < 0 && test.z < 0 && test.y<0) {
				dek_p_a = i; break;
			}
		}
	}

 	for (int j = 0; j < meloi[dek].size(); j++) {mprosta_aristero.push_back(meloi[dek][j]);}

 	for (int j = 0; j < meloi[dek_m_d].size(); j++) {mprosta_deksi.push_back(meloi[dek_m_d][j]);}

 	for (int j = 0; j < meloi[dek_p_d].size(); j++) { pisw_deksi.push_back(meloi[dek_p_d][j]); }

 	for (int j = 0; j < meloi[dek_p_a].size(); j++) { pisw_aristero.push_back(meloi[dek_p_a][j]); }



	mprosta_aristero_peris=peristorfi(mprosta_aristero);
	simeia_komatiou(mprosta_aristero, mprosta_aristero_poi);


	pisw_aristero_peris = peristorfi(pisw_aristero);
	simeia_komatiou(pisw_aristero, pisw_aristero_poi);

	pisw_deksi_peris = peristorfi(pisw_aristero);
	simeia_komatiou(pisw_deksi, pisw_deksi_poi);

	 mprosta_deksi_peris = peristorfi(mprosta_deksi); 
	simeia_komatiou(mprosta_deksi, mprosta_deksi_poi);

	//peristrofis = mprosta_aristero_peris;
	vector<vec> reeb_point;
	
	for (int i = 0; i < reeb_lines.size(); i++) {

		LineSeg3D line2 = reeb_lines[i];
		vec start2, end2;
		start2.x = line2.x1;
		start2.y = line2.y1;
		start2.z = line2.z1;
		end2.x = line2.x2;
		end2.y = line2.y2;
		end2.z = line2.z2;
		bool f1 = true, f2 = true;
		for (int j = 0; j < reeb_point.size(); j++) {

			if (start2.x == reeb_point[j].x && start2.y == reeb_point[j].y && start2.z == reeb_point[j].z) { f1 = false; }
			if (end2.x == reeb_point[j].x && end2.y == reeb_point[j].y && end2.z == reeb_point[j].z) { f2 = false; }


		}
		if (f1)reeb_point.push_back(start2);
		if (f2)reeb_point.push_back(end2);
		f1 = true; f2 = true;
	}

 


	for (int i = 0; i < m_model.getVertices().size(); i++) {
		float dist = inf;
		vec deik;
		for (int j = 0; j < reeb_point.size(); j++) {
			if (dist > reeb_point[j].Distance(m_model.getVertices()[i])) {

				dist = reeb_point[j].Distance(m_model.getVertices()[i]);
				deik = reeb_point[j];
			}
		}
		antistoixeia.push_back(make_pair(deik, dist));

	}
	
	for (int j = 0; j < reeb_point.size(); j++) {
		for (int i = 0; i < cm_antikeimenwn.size(); i++) {
			if (reeb_point[j].x == cm_antikeimenwn[i].x  && reeb_point[j].y == cm_antikeimenwn[i].y && reeb_point[j].z == cm_antikeimenwn[i].z) {
				for (int k = 0; k < antikeimena_mesh[i].getVertices().size(); k++) {
					vec ant = antikeimena_mesh[i].getVertices()[k];
					for (int l = 0; l < m_model.getVertices().size(); l++) {
						vec mod = m_model.getVertices()[l];
						if (ant.x == mod.x && ant.y==mod.y && ant.z==mod.z) {
							antistoixeia[l].first = cm_antikeimenwn[i];
							antistoixeia[l].second = cm_antikeimenwn[i].Distance(mod);
						}
					}
				}
			}
		}
	}
	 
	vec oura_peristrofis = peristorfi(oura);
	peristrofis = oura_peristrofis;
	simeia_komatiou(oura, oura_point);
 
	
}
void Mesh3DScene::simeia_komatiou(std::vector<vvr::LineSeg3D> komati, std::vector<math::vec>& points)
{

	for (int i = 0; i < komati.size(); i++) {
		LineSeg3D line2 = komati[i];
		vec start2, end2;
		start2.x = line2.x1;
		start2.y = line2.y1;
		start2.z = line2.z1;
		end2.x = line2.x2;
		end2.y = line2.y2;
		end2.z = line2.z2;
		bool f1 = true, f2 = true;
		for (int j = 0; j < points.size(); j++) {
			if (start2.x == points[j].x && start2.y == points[j].y && start2.z == points[j].z) { f1 = false; }
			if (end2.x == points[j].x && end2.y == points[j].y && end2.z == points[j].z) { f2 = false; }
		}
		if (f1)points.push_back(start2);
		if (f2)points.push_back(end2);
		f1 = true; f2 = true;
	} 
}
void Mesh3DScene::fliptale(vec point, float moires ){
	for (int i = 0; i < m_model.getVertices().size(); i++) {
		for (int j = 0; j < oura_point.size(); j++) {
			if (oura_point[j].x == antistoixeia[i].first.x && oura_point[j].y == antistoixeia[i].first.y && oura_point[j].z == antistoixeia[i].first.z) {

				float z = m_model.getVertices()[i].z - point.z;
				float y = m_model.getVertices()[i].y - point.y;
				m_model.getVertices()[i].y = y*cos(moires) - z*sin(moires);
				m_model.getVertices()[i].z = y*sin(moires) + z*cos(moires);
				m_model.getVertices()[i].z = m_model.getVertices()[i].z + point.z;
				m_model.getVertices()[i].y = m_model.getVertices()[i].y + point.y;

			}
		}

	}
}
void Mesh3DScene::flipleg(vec point, float moires,vector<vec> simeia)
{
	for (int i = 0; i < m_model.getVertices().size(); i++) {
		for (int j = 0; j < simeia.size(); j++) {
			if (simeia[j].x == antistoixeia[i].first.x && simeia[j].y == antistoixeia[i].first.y && simeia[j].z == antistoixeia[i].first.z) {
				float x = m_model.getVertices()[i].x - point.x;
				float y = m_model.getVertices()[i].y - point.y;

				m_model.getVertices()[i].x = x*cos(moires) - y*sin(moires);
				m_model.getVertices()[i].y = x*sin(moires) + y*cos(moires);

				m_model.getVertices()[i].x = m_model.getVertices()[i].x + point.x;
				m_model.getVertices()[i].y = m_model.getVertices()[i].y + point.y;
			}
		}
	}
}
vec Mesh3DScene::peristorfi(vector<LineSeg3D> komati) {
	vec peri_p;
	for (int i = 0; i < komati.size(); i++) {
		LineSeg3D line = komati[i];

		vector<vec> point;
		point.push_back(vec(line.x1, line.y1, line.z1));
		point.push_back(vec(line.x2, line.y2, line.z2));


		bool flag = false;
		for (int k = 0; k < 2; k++) {
			int metritis = 0;
			for (int j = 0; j < komati.size(); j++) {
				LineSeg3D line2 = komati[j];
				vec start2;
				start2.x = line2.x1;
				start2.y = line2.y1;
				start2.z = line2.z1;

				vec end2;
				end2.x = line2.x2;
				end2.y = line2.y2;
				end2.z = line2.z2;
				if (j != i) {
					if (point[k].x == start2.x && point[k].y == start2.y && point[k].z == start2.z) { metritis++; }

					else 	if (point[k].x == end2.x && point[k].y == end2.y && point[k].z == end2.z) { metritis++; }
				}
			}
			if (metritis == 0) {
				for (int g = 0; g < telika_akraia_si.size(); g++) {

					if (telika_akraia_si[g].x == point[k].x && telika_akraia_si[g].y == point[k].y && telika_akraia_si[g].z == point[k].z) { flag = true; }
				}
				if (flag == false) {
					peri_p = point[k];
				}
			}
		}
	}
	return peri_p;
}
void Mesh3DScene::curvuture() {

	vector<vec> simeia;
	for (int j = 0; j < m_model.getTriangles().size(); j++) {
		vvr::Triangle tris = m_model.getTriangles()[j];
		vec a = tris.v1();
		vec b = tris.v2();
		vec c = tris.v3();
		pointset.push_back(a);
		pointset.push_back(b);
		pointset.push_back(c);

		float alpha = a.Distance(b);
		float bita = a.Distance(c);
		float cita = c.Distance(b);

		if (alpha > bita && alpha > cita) {
			float tempa = alpha;
			float tempb = bita;
			float tempc = cita;
			cita = tempa;
			alpha = tempc;
		}
		else 	if (bita > alpha && bita > cita) {
			float tempa = alpha;
			float tempb = bita;
			float tempc = cita;
			cita = tempb;
			bita = tempc;
		}
		if (alpha*alpha + bita*bita <= cita*cita) {
			tris.getCenter();
			pointset3.push_back(tris.getCenter());
			simeia.push_back(tris.getCenter());
		}
		else {

			float ac = a.Distance(c);
			float ab = a.Distance(b);
			vec temp1 = Cross(Cross(vec(b.x - a.x, b.y - a.y, b.z - a.z), vec(c.x - a.x, c.y - a.y, c.z - a.z)), vec(b.x - a.x, b.y - a.y, b.z - a.z));
			vec temp2 = Cross(vec(c.x - a.x, c.y - a.y, c.z - a.z), Cross(vec(b.x - a.x, b.y - a.y, b.z - a.z), vec(c.x - a.x, c.y - a.y, c.z - a.z)));
			a.Cross(b);

			temp1.x = pow(ac, 2)*temp1.x;
			temp1.y = pow(ac, 2)*temp1.y;
			temp1.z = pow(ac, 2)*temp1.z;

			temp2.x = pow(ab, 2)*temp2.x;
			temp2.y = pow(ab, 2)*temp2.y;
			temp2.z = pow(ab, 2)*temp2.z;


			temp1.x = temp1.x + temp2.x;;
			temp1.y = temp1.y + temp2.y;
			temp1.z = temp1.z + temp2.z;

			vec temp3 = Cross(vec(b.x - a.x, b.y - a.y, b.z - a.z), vec(c.x - a.x, c.y - a.y, c.z - a.z));

			float paranomasis = 2 * (pow(temp3.x, 2) + pow(temp3.y, 2) + pow(temp3.z, 2));

			temp1.x = temp1.x / paranomasis;
			temp1.y = temp1.y / paranomasis;
			temp1.z = temp1.z / paranomasis;

			a.x = a.x + temp1.x;
			a.y = a.y + temp1.y;
			a.z = a.z + temp1.z;

			pointset3.push_back(a);
			simeia.push_back(a);
		
		}

	}

 

	  max_curv=0;
	 min_curv=inf;




	for (int i = 0; i < m_model.getVertices().size(); i++) {//1; i++) {//
		vector<std::pair<vvr::Triangle, int>> trigwna;
		for (int j = 0; j < m_model.getTriangles().size(); j++) {
			vvr::Triangle tris = m_model.getTriangles()[j];
			if (tris.vi1 == i || tris.vi2 == i || tris.vi3 == i) {
				trigwna.push_back(make_pair(tris, j));

			}

		}


		float A = 0;
		vector<std::pair<LineSeg3D, float>> grammes;

		for (int j = 0; j < trigwna.size(); j++) {
			vec a, b, c;
			if (i == trigwna[j].first.vi1) {
				a = trigwna[j].first.v1();
				b = trigwna[j].first.v2();
				c = trigwna[j].first.v3();

			}
			else if (i == trigwna[j].first.vi2) {
				a = trigwna[j].first.v2();
				b = trigwna[j].first.v1();
				c = trigwna[j].first.v3();

			}
			else if (i == trigwna[j].first.vi3) {
				a = trigwna[j].first.v3();
				b = trigwna[j].first.v1();
				c = trigwna[j].first.v2();

			}

			A = A + embadon(Triangle3D(a.x, a.y, a.z, simeia[trigwna[j].second].x, simeia[trigwna[j].second].y, simeia[trigwna[j].second].z, (a.x + b.x) / 2, (a.y + b.y) / 2, (a.z + b.z) / 2));
			A = A + embadon(Triangle3D(a.x, a.y, a.z, simeia[trigwna[j].second].x, simeia[trigwna[j].second].y, simeia[trigwna[j].second].z, (a.x + c.x) / 2, (a.y + c.y) / 2, (a.z + c.z) / 2));



			grammes.push_back(make_pair(LineSeg3D(a.x, a.y, a.z, b.x, b.y, b.z), 0));
			grammes.push_back(make_pair(LineSeg3D(a.x, a.y, a.z, c.x, c.y, c.z), 0));
		}


			//diagrafi diplwn grammwn
			for (int k = 0; k < grammes.size(); k++) {
				LineSeg3D line1 = grammes[k].first;
				for (int j = 0; j < grammes.size(); j++) {
					LineSeg3D line2 = grammes[j].first;
					if (k != j) {
						if ((line1.x1 == line2.x1 && line1.y1 == line2.y1 && line1.z1 == line2.z1  && line1.x2 == line2.x2 && line1.y2 == line2.y2 && line1.z2 == line2.z2) || (line1.x1 == line2.x2 && line1.y1 == line2.y2 && line1.z1 == line2.z2  && line1.x2 == line2.x1 && line1.y2 == line2.y1 && line1.z2 == line2.z1)) {
							grammes[k].first.x1 = inf;
							grammes[k].first.y1 = inf;
							grammes[k].first.z1 = inf;
							grammes[k].first.x2 = inf;
							grammes[k].first.y2 = inf;
							grammes[k].first.z2 = inf;
						}
					}
				}
			}
			for (int k = 0; k < grammes.size(); k++)
			{
				if (grammes[k].first.x1 == inf && grammes[k].first.y1 == inf && grammes[k].first.z1 == inf && grammes[k].first.x2 == inf && grammes[k].first.y2 == inf && grammes[k].first.z2 == inf) {
					grammes.erase(grammes.begin() + k);
					k--;
				}
			}


			for (int k = 0; k < grammes.size(); k++) {
				vec a;
				a.x = grammes[k].first.x1;
				a.y = grammes[k].first.y1;
				a.z = grammes[k].first.z1;

				vec b;
				b.x = grammes[k].first.x2;
				b.y = grammes[k].first.y2;
				b.z = grammes[k].first.z2;
				for (int t = 0; t < trigwna.size(); t++) {
					vvr::Triangle tris = trigwna[t].first;
					if ((a.x == tris.v1().x && a.y == tris.v1().y && a.z == tris.v1().z && b.x == tris.v2().x && b.y == tris.v2().y && b.z == tris.v2().z) || (b.x == tris.v1().x && b.y == tris.v1().y && b.z == tris.v1().z && a.x == tris.v2().x && a.y == tris.v2().y && a.z == tris.v2().z)) {

						vec dianisma1;
						dianisma1.x = tris.v1().x - tris.v3().x;
						dianisma1.y = tris.v1().y - tris.v3().y;
						dianisma1.z = tris.v1().z - tris.v3().z;

						vec dianisma2;
						dianisma2.x = tris.v2().x - tris.v3().x;
						dianisma2.y = tris.v2().y - tris.v3().y;
						dianisma2.z = tris.v2().z - tris.v3().z;
						grammes[k].second = grammes[k].second + Dot(dianisma1, dianisma2) / (dianisma1.Length()*dianisma2.Length());

					}
					else if ((a.x == tris.v1().x && a.y == tris.v1().y && a.z == tris.v1().z && b.x == tris.v3().x && b.y == tris.v3().y && b.z == tris.v3().z) || (b.x == tris.v1().x && b.y == tris.v1().y && b.z == tris.v1().z && a.x == tris.v3().x && a.y == tris.v3().y && a.z == tris.v3().z)) {
						vec dianisma1;
						dianisma1.x = tris.v1().x - tris.v2().x;
						dianisma1.y = tris.v1().y - tris.v2().y;
						dianisma1.z = tris.v1().z - tris.v2().z;

						vec dianisma2;
						dianisma2.x = tris.v3().x - tris.v2().x;
						dianisma2.y = tris.v3().y - tris.v2().y;
						dianisma2.z = tris.v3().z - tris.v2().z;
						grammes[k].second = grammes[k].second + Dot(dianisma1, dianisma2) / (dianisma1.Length()*dianisma2.Length());
					}
					else if ((a.x == tris.v3().x && a.y == tris.v3().y && a.z == tris.v3().z && b.x == tris.v2().x && b.y == tris.v2().y && b.z == tris.v2().z) || (b.x == tris.v3().x && b.y == tris.v3().y && b.z == tris.v3().z && a.x == tris.v2().x && a.y == tris.v2().y && a.z == tris.v2().z)) {

						vec dianisma1;
						dianisma1.x = tris.v3().x - tris.v1().x;
						dianisma1.y = tris.v3().y - tris.v1().y;
						dianisma1.z = tris.v3().z - tris.v1().z;

						vec dianisma2;
						dianisma2.x = tris.v2().x - tris.v1().x;
						dianisma2.y = tris.v2().y - tris.v1().y;
						dianisma2.z = tris.v2().z - tris.v1().z;
						grammes[k].second = grammes[k].second + Dot(dianisma1, dianisma2) / (dianisma1.Length()*dianisma2.Length());
					}

				}
			}



			vector<vec> vec_athroisma;
			for (int k = 0; k < grammes.size(); k++) {
				vec p;
				vec x;
				if (grammes[k].first.x1 == m_model.getVertices()[i].x && grammes[k].first.y1 == m_model.getVertices()[i].y && grammes[k].first.z1 == m_model.getVertices()[i].z) {
					p = m_model.getVertices()[i];
					x.x = grammes[k].first.x2;
					x.y = grammes[k].first.y2;
					x.z = grammes[k].first.z2;

				}
				else if (grammes[k].first.x2 == m_model.getVertices()[i].x && grammes[k].first.y2 == m_model.getVertices()[i].y && grammes[k].first.z2 == m_model.getVertices()[i].z) {
					p = m_model.getVertices()[i];
					x.x = grammes[k].first.x1;
					x.y = grammes[k].first.y1;
					x.z = grammes[k].first.z1;

				}


				vec_athroisma.push_back(vec((x.x - p.x)*grammes[k].second, (x.y - p.y)*grammes[k].second, (x.z - p.z)*grammes[k].second));

			}



			vec teliko = vec(0, 0, 0);
			for (int k = 0; k < vec_athroisma.size(); k++) {

				teliko.x = teliko.x + vec_athroisma[k].x;
				teliko.y = teliko.y + vec_athroisma[k].y;
				teliko.z = teliko.z + vec_athroisma[k].z;

			}




	//		echo(i);
			
			
			 

			if (teliko.Length() / (4 * A) > max_curv)max_curv = teliko.Length() / (4 * A);
			if (teliko.Length() / (4 * A) < min_curv)min_curv = teliko.Length() / (4 * A);


		 


			curvuture_vaues.push_back(teliko.Length() / (4 * A));






		}




		int g = 0;
		g = 1;
	}





