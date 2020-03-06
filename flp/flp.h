#pragma once
#include <Eigen/Geometry>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/exterior_property.hpp>
#include <boost/graph/johnson_all_pairs_shortest.hpp>
#include <boost/graph/graphviz.hpp>
#include <vector>
#include <cmath>
#include <string>
#include <iostream>

namespace flp
{

  //-----------------------------------------------------------------------
  // basic statistics

  struct Stats
  {
    Stats() : sum(0.0), avg(0.0), std(0.0), min(std::numeric_limits<double>::infinity()), max(-std::numeric_limits<double>::infinity()), count(0.0), M2(0.0) {}
    void update(double val) {
      // welford online stddev
      count = count + 1;
      double delta = val - avg;
      avg = avg + delta / count;
      double delta2 = val - avg;
      M2 = M2 + delta * delta2;
      std = sqrt(M2/count);
      // others
      sum += val;
      if (val < min) min = val;
      if (val > max) max = val;
    }
    friend std::ostream& operator<<(std::ostream& ss, const Stats& stats) {
      ss << "sum=" << stats.sum << ", avg=" << stats.avg << ", std=" << stats.std << ", min=" << stats.min << ", max=" << stats.max << " ";
      return ss;
    }
    double sum, avg, std, min, max, count, M2;
  };

  //-----------------------------------------------------------------------
  // geometry

  typedef ::std::vector<double> Vector;

  struct Point {
    Point(const Eigen::Vector3d& xyz) : p(xyz), label(0), friction(-1.0), roughness(-1.0), traversability(-1.0) {}
    Point(double x, double y, double z) : p(x,y,z), label(0), friction(-1.0), roughness(-1.0), traversability(-1.0) {}
    Eigen::Vector3d p;
    unsigned int label;
    double friction;
    double roughness;
    double traversability;
  };

  struct Transform {
    double x, y, z, qx, qy, qz, qw;
    Transform() : x(0), y(0), z(0), qx(0), qy(0), qz(0), qw(0) {}
    Transform(double xx, double yy, double zz, double R, double P, double Y) : x(xx), y(yy), z(zz) { setRPY(R,P,Y); }
    void setRPY(double roll, double pitch, double yaw) {
      // rotate around z-axis (yaw), then y-axis (pitch), then x-axis (roll)
      Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
      qx = q.x();
      qy = q.y();
      qz = q.z();
      qw = q.w();
    }
    void getRPY(double& roll, double& pitch, double& yaw) const {
      Eigen::Quaterniond q (qw, qx, qy, qz); // NOTE: not using Eigen's eulerAngles() because they return the first angle within [0,pi] which is not what I want, yaw in [-pi,pi[
      // roll (x-axis rotation)
      double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
      double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
      roll = atan2(sinr_cosp, cosr_cosp);
      // pitch (y-axis rotation)
      double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
      if (fabs(sinp) >= 1)
      pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
      else
      pitch = asin(sinp);
      // yaw (z-axis rotation)
      double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
      double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
      yaw = atan2(siny_cosp, cosy_cosp);
    }
    Eigen::Transform<double,3,Eigen::Affine> getEigenTransform() const {
      Eigen::Quaterniond eiq(qw, qx, qy, qz);
      Eigen::Transform<double,3,Eigen::Affine> eiT = Eigen::Translation<double,3>(Eigen::Vector3d(x,y,z)) * eiq;
      return eiT;
    }
  };

  //-----------------------------------------------------------------------
  // geometric primitives

  enum PrimitiveType {
    PrimitiveTypeSphere,
    PrimitiveTypeCylinder,
    PrimitiveTypeBox,
    PrimitiveTypeSolid
  };
  struct Sphere {
    Eigen::Vector3d center;
    double radius;
    void transform(Eigen::Transform<double,3,Eigen::Affine> T) {
      center = T * center;
    }
  };
  struct Cylinder {
    Eigen::Vector3d center;
    double radius;
    double height;
    void transform(Eigen::Transform<double,3,Eigen::Affine> T) {
      center = T * center;
    }
  };
  struct Box {
    //Eigen::Transform<double,3,Eigen::Affine> T;
    Eigen::Vector3d extents;
    void transform(Eigen::Transform<double,3,Eigen::Affine> T2) {
      //T = T * T2;
    }
  };
  struct Polygon {
    std::vector<Eigen::Vector3d> vertices;
    void transform(Eigen::Transform<double,3,Eigen::Affine> T) {
      for (unsigned int i = 0; i < vertices.size(); i++)
        vertices[i] = T * vertices[i];
    }
  };
  struct Solid {
    std::vector<Polygon> polygons;
    void transform(Eigen::Transform<double,3,Eigen::Affine> T) {
      for (unsigned int i = 0; i < polygons.size(); i++)
        polygons[i].transform(T);
    }
  };
  struct Primitive {
    PrimitiveType type;
    std::string name;
    Solid solid;
    Sphere sphere;
    Cylinder cylinder;
    Box box;
    void transform(Eigen::Transform<double,3,Eigen::Affine> T) {
      if (type == PrimitiveTypeSphere)
        sphere.transform(T);
      else if (type == PrimitiveTypeCylinder)
        cylinder.transform(T);
      else if (type == PrimitiveTypeBox)
        box.transform(T);
      else if (type == PrimitiveTypeSolid)
        solid.transform(T);
    }
    double expectedNumberOfPoints(double resolution) const {
      if (type == PrimitiveTypeSphere)
        return M_PI * sphere.radius * sphere.radius / (resolution * resolution);
      if (type == PrimitiveTypeCylinder)
        return M_PI * cylinder.radius * cylinder.radius / (resolution * resolution);
      std::cout << "WARNING: expectedNumberOfPoints() not implemeted for this primitive type\n";
      return 0;
    }
  };

  //-----------------------------------------------------------------------
  // graphs

  class Graph
  {
  public:
    struct Vertex {
      std::string name;
      std::vector< std::vector<double> > data;
    };
    struct Edge {
      Edge() : weight(0), color("black") { update(); }
      Edge(size_t w) : weight(w), color("black") { update(); }
      Edge(size_t w, const std::vector<double>& p) : weight(w), parameters(p), color("black") { update(); }
      void update() {
        std::stringstream ss;
        ss << weight << " ";
        for (unsigned int i = 0; i < parameters.size(); i++)
          ss << parameters[i] << " ";
        label = ss.str();
      }
      size_t weight;
      std::vector<double> parameters;
      std::string label;
      std::string color;
    };
    typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, Vertex, Edge> Type;
    typedef boost::graph_traits<Type>::vertex_descriptor vertex_descriptor;
    typedef boost::graph_traits<Type>::edge_descriptor edge_descriptor;
    Graph() : V_(0), E_(0) {
    }
    void addEdge(int i, int j, size_t weight) {
      boost::add_edge(i, j, Edge(weight), graph_);
    }
    void addEdge(int i, int j, size_t weight, const std::vector<double>& parameters) {
      boost::add_edge(i, j, Edge(weight, parameters), graph_);
    }
    void removeEdge(int i, int j) {
      boost::remove_edge(i, j, graph_);
    }
    void update() {
      // get shortest distances by johnson_all_pairs_shortest_paths
      V_ = boost::num_vertices(graph_);
      E_ = boost::num_edges(graph_);
      boost::exterior_vertex_property<Type, size_t>::matrix_type d(V_);
      boost::johnson_all_pairs_shortest_paths(graph_, d, boost::weight_map(boost::get(&Edge::weight, graph_)));
      // store
      dist_ = Eigen::MatrixXi(V_,V_);
      for (size_t i = 0; i < V_; i++)
        for (size_t j = 0; j < V_; j++)
          dist_(i,j) = d[i][j];
    }
    void transitiveReduction() {
      // Harry Hsu, "An algorithm for finding a minimal equivalent graph of a digraph.", 1975
      // https://stackoverflow.com/a/6702198
      for (int j = 0; j < V_; ++j) {
        for (int i = 0; i < V_; ++i) {
          // if a path exists between i and j
          if (dist_(i,j) > 0) {
            for (int k = 0; k < V_; ++k) {
              // and also between j and k
              if (dist_(j,k) > 0) {
                // remove i-k
                removeEdge(i,k);
                update();
              }
            }
          }
        }
      }
    }
    int getNumberOfNodes() const {
      return V_;
    }
    int getNumberOfEdges() const {
      return E_;
    }
    double getDistance(int i, int j) const {
      return dist_(i, j);
    }
    std::vector<int> getNeighbors(int node) const {
      std::vector<int> nei;
      nei.reserve(V_);
      // get adjacent vertices
      Type::adjacency_iterator vi, vi_end;
      for (boost::tie(vi, vi_end) = boost::adjacent_vertices(node, graph_); vi != vi_end; ++vi) {
        nei.push_back(*vi);
      }
      return nei;
    }
    std::vector<int> getShortestPath(int start, int goal) const {
      // search
      std::vector<vertex_descriptor> p(V_);
      std::vector<int> d(V_);
      vertex_descriptor s = boost::vertex(start, graph_);
      vertex_descriptor g = boost::vertex(goal, graph_);
      boost::dijkstra_shortest_paths(graph_, s, boost::predecessor_map(&p[0]).distance_map(&d[0]).weight_map(boost::get(&Edge::weight, graph_)));
      // get path
      vertex_descriptor i = g;
      std::vector<int> path;
      while (i != s) {
        path.push_back(i);
        i = p[i];
        if (i == g) return std::vector<int>(); // failure
      }
      path.push_back(s);
      // reverse order to get start to goal
      std::reverse(path.begin(), path.end());
      return path;
    }
    void setColor(const std::string& color) {
      boost::graph_traits<Type>::edge_iterator ei, ei_end;
      for (boost::tie(ei, ei_end) = boost::edges(graph_); ei != ei_end; ++ei)
        graph_[*ei].color = color;
    }
    void setColorEdge(int i, int j, const std::string& color) {
      std::pair<edge_descriptor, bool> edge = boost::edge(i, j, graph_);
      if (!edge.second) return;
      graph_[edge.first].color = color;
    }
    bool setColorPath(const std::vector<int>& path, const std::string& color) {
      if (path.size() < 2) return false;
      for (unsigned int i = 0; i < path.size()-1; i++) {
        std::pair<edge_descriptor, bool> edge = boost::edge(path[i], path[i+1], graph_);
        if (!edge.second) return false;
        graph_[edge.first].color = color;
      }
      return true;
    }
    std::vector<int> getEdgesWithColor(const std::string& color) const {
      std::vector<int> edges;
      boost::graph_traits<Type>::edge_iterator ei, ei_end;
      for (boost::tie(ei, ei_end) = boost::edges(graph_); ei != ei_end; ++ei) {
        if (graph_[*ei].color == color) {
          edges.push_back(boost::source(*ei, graph_));
          edges.push_back(boost::target(*ei, graph_));
        }
      }
      return edges;
    }
    int getEdgesVectorIndex(int i, int j) const {
      int index = 0;
      boost::graph_traits<Type>::edge_iterator ei, ei_end;
      for (boost::tie(ei, ei_end) = boost::edges(graph_); ei != ei_end; ++ei) {
        const Edge& e = graph_[*ei];
        int edgei = boost::source(*ei, graph_);
        int edgej = boost::target(*ei, graph_);
        if (edgei == i && edgej == j)
          return index;
        else
          index += 1 + e.parameters.size();
      }
      return -1;
    }
    std::vector<double> getEdgesVector() const {
      std::vector<double> vec;
      boost::graph_traits<Type>::edge_iterator ei, ei_end;
      for (boost::tie(ei, ei_end) = boost::edges(graph_); ei != ei_end; ++ei) {
        const Edge& e = graph_[*ei];
        vec.push_back(e.weight);
        for (unsigned int i = 0; i < e.parameters.size(); i++)
          vec.push_back(e.parameters[i]);
      }
      return vec;
    }
    bool setEdgesVector(const std::vector<double>& vec) {
      unsigned int k = 0;
      boost::graph_traits<Type>::edge_iterator ei, ei_end;
      // check if size is the same
      for (boost::tie(ei, ei_end) = boost::edges(graph_); ei != ei_end; ++ei)
        k += 1 + graph_[*ei].parameters.size();
      if (k != vec.size())
        return false;
      // set edge properties
      k = 0;
      for (boost::tie(ei, ei_end) = boost::edges(graph_); ei != ei_end; ++ei) {
        Edge& e = graph_[*ei];
        e.weight = vec[k++];
        for (unsigned int i = 0; i < e.parameters.size(); i++)
          e.parameters[i] = vec[k++];
        e.update();
      }
      return true;
    }
    std::vector<double> getEdgeParameters(int i, int j) const {
      std::pair<edge_descriptor, bool> edge = boost::edge(i, j, graph_);
      if (!edge.second) return std::vector<double>();
      return graph_[edge.first].parameters;
    }
    std::string getNodeName(int i) const {
      return graph_[i].name;
    }
    void setNodeName(int i, const std::string& name) {
      graph_[i].name = name;
    }
    std::vector< std::vector<double> > getNodeData(int i) const {
      return graph_[i].data;
    }
    void setNodeData(int i, const std::vector< std::vector<double> >& data) {
      graph_[i].data = data;
    }
    void savedot(const std::string& filename) {
      std::ofstream file(filename);
      boost::dynamic_properties dp;
      dp.property("node_id", boost::get(boost::vertex_index, graph_));
      dp.property("label", boost::get(&Vertex::name, graph_));
      dp.property("label", boost::get(&Edge::label, graph_));
      dp.property("color", boost::get(&Edge::color, graph_));
      boost::write_graphviz_dp(file, graph_, dp);
    }
    void savepdf(const std::string& filename) {
      std::string dotfilename = filename + std::string(".dot");
      savedot(dotfilename);
      // convert to pdf
      std::string cmd1 = std::string("dot -Tpdf ") + dotfilename + std::string(" -o ") + filename;
      int res1 = system(cmd1.c_str());
      // delete old .dot
      //std::string cmd2 = std::string("rm ") + dotfilename;
      //int res2 = system(cmd2.c_str());
    }
    void savepng(const std::string& filename) {
      std::string dotfilename = filename + std::string(".dot");
      savedot(dotfilename);
      // convert to pdf
      std::string cmd1 = std::string("dot -Tpng ") + dotfilename + std::string(" -o ") + filename;
      int res1 = system(cmd1.c_str());
      // delete old .dot
      //std::string cmd2 = std::string("rm ") + dotfilename;
      //int res2 = system(cmd2.c_str());
    }
  protected:
    int V_;
    int E_;
    Type graph_;
    Eigen::MatrixXi dist_;
  };

};

