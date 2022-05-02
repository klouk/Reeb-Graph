#include <VVRScene/canvas.h>
#include <VVRScene/mesh.h>
#include <VVRScene/settings.h>
#include <VVRScene/utils.h>
#include <MathGeoLib.h>
#include <iostream>
#include <fstream>
#include <cstring>
#include <string>
#include <set>
#include "symmetriceigensolver3x3.h"


#define FLAG_SHOW_AXES       1
#define FLAG_SHOW_WIRE       2
#define FLAG_SHOW_SOLID      4
#define FLAG_SHOW_NORMALS    8
#define FLAG_SHOW_PLANE     16
#define FLAG_SHOW_AABB      32
#define FLAG_SHOW_DUAL_GRAPH 64

#define FLAG_SHOW_NEXT 128 
 

std::vector<math::Triangle> tris;
float embadon(vvr::Triangle3D trig);
bool Line_Plane_Intersection(vec& contact, vvr::LineSeg3D line, Plane &plane);
class Mesh3DScene : public vvr::Scene
{
public:
    Mesh3DScene();
    const char* getName() const { return "3D Scene"; }
    void keyEvent(unsigned char key, bool up, int modif) override;
    void arrowEvent(vvr::ArrowDir dir, int modif) override;

private:
    void draw() override;
    void reset() override;
    void resize() override;
	void mousePressed(int x, int y, int modif) override;
	void FindAdjacentTriangles(int i, std::vector<int> &a);
	void dualgraph();
	void Dijkstra(int src, float &max,float &min);
	void GeodesicDistance();
	void Poligwnopoisi();
	void Convex3D();
	void piramida();
	void Euresi_path(std::vector<vvr::LineSeg3D> & path);
	void search_grafo_for_sigrsouseis(std::vector<std::pair<int, int>> grafos, std::vector<int>& intex,int intex_simeiou_pointset);
	void reeb_graph_height();
	void reeb_graph_geodesic_distances();
	void antikeimena(std::vector<std::pair<int, int>> intexes, std::vector<int> &processed,int thesi,int metritis_antikeimenwn);
	bool FindAdjacentTriangle(vvr::Mesh mesh, vec p1, vec p2);
	bool FindAdjacentTriangle(std::vector<std::pair<int, int>> intexes, vec p1, vec p2, int & tri_adj_index, int thesi);
	void simplification();
	void partition();
	void telika_akria(int i );
	void motion();
	void fliptale(math::vec  point, float moires);
	void flipleg(math::vec  point, float moires, std::vector<math::vec> simeia);
	vec peristorfi(std::vector<vvr::LineSeg3D > komati);
	void simeia_komatiou(std::vector<vvr::LineSeg3D> komati, std::vector<math::vec> & points);
	void curvuture();
	 
private:
	int ena;
    int m_style_flag;
    float m_plane_d;
    vvr::Canvas2D m_canvas;
    vvr::Colour m_obj_col;
    vvr::Mesh m_model_original, m_model;
    vvr::Box3D m_aabb;
    math::vec m_center_mass;
    math::vec m_pca_cen;
    math::vec m_pca_dir;
    math::Plane m_plane;
    std::vector<int> m_intersections;
	std::vector<vec>  kentra_dual_graph;
	std::vector<std::vector<float>>  graph;
	vvr::Point3D cm_piramidas;
	std::vector<vec>  graph2;
	std::vector<vec>  graph3;
	std::vector<vvr::LineSeg3D> dual_graph;
	std::vector<vvr::Triangle3D>  valid_trigwna_piramidas;
	std::vector<vvr::Triangle3D>  trigwna_piramidas;
	std::vector<vec> pointset; std::vector<vec> pointset3;
	std::vector<std::vector<float>>  graphfinal2;
	std::vector<float>  geodesiakes;
	std::vector<vec> piramida_points;
	vvr::Mesh convexhull;
	vvr::Mesh greenMesh;
	vvr::Mesh simplified;
	vvr::Mesh redMesh;
	std::vector<vvr::Mesh> antikeimena_mesh;
	float max_g, min_g;
	Plane plane, plane_t;
	std::vector<vvr::LineSeg3D> reeb_lines;
	std::vector<vec> cm_antikeimenwn;
	std::vector <vec> akraia_simeia;
	std::vector<std::pair<vec, float>> geodesic_akraiwn;
	std::vector<std::vector<int>> parallilo_geodesiakes;
	std::vector<std::vector<vvr::Triangle3D>>  kommatia;
	std::vector<int> kati;
	std::vector <float> geodesiakes_points;
	std::vector<std::vector<vec>> telika_akraia_simeia;
	std::vector<vec> telika_akraia_si;
	std::vector<std::pair<int, int>> aris_deks;
	std::vector<vvr::LineSeg3D> oura;
	std::vector<std::pair<vec, float>> antistoixeia;
	math::vec peristrofis;
	std::vector<vec> oura_point;
	std::vector<std::vector<vvr::LineSeg3D>> meloi;
	std::vector<vvr::LineSeg3D > mprosta_aristero;
	std::vector<vvr::LineSeg3D > mprosta_deksi;
	std::vector<vvr::LineSeg3D > pisw_aristero;
	std::vector<vvr::LineSeg3D > pisw_deksi;
	std::vector<std::vector<math::vec>> komatia_ant;
	std::vector<std::pair<int, int>> antoisti;
	std::vector<int> kanonika;
	std::vector<std::vector<int>> intex_antikeinwn;
	math::vec mprosta_aristero_peris;
	math::vec mprosta_deksi_peris;
	math::vec pisw_aristero_peris;
	math::vec pisw_deksi_peris;
	std::vector<math::vec> mprosta_aristero_poi;
	std::vector<math::vec> mprosta_deksi_poi;
	std::vector<math::vec> pisw_aristero_poi;
	std::vector<math::vec> pisw_deksi_poi;
	std::vector<std::vector<vvr::Triangle>> trigwna_antikeimenwn;
	std::vector<vvr::LineSeg3D> aksonas;
	std::vector<float> curvuture_vaues;
	float max_curv ;
	float min_curv  ;
	float moires = 100;
	int qw= 1;
};







