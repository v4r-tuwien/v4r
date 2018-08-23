#include <v4r/common/pcl_visualization_utils.h>

namespace po = boost::program_options;

namespace v4r {
std::vector<int> pcl_visualizer::visualization_framework(pcl::visualization::PCLVisualizer &vis, size_t number_of_views,
                                                         size_t number_of_subwindows_per_view,
                                                         const std::vector<std::string> &title_subwindows) {
  vis.removeAllPointClouds();
  vis.removeAllShapes();
  std::vector<int> viewportNr(number_of_views * number_of_subwindows_per_view, 0);

  for (size_t i = 0; i < number_of_views; i++) {
    for (size_t j = 0; j < number_of_subwindows_per_view; j++) {
      vis.createViewPort(float(i) / number_of_views, float(j) / number_of_subwindows_per_view,
                         (float(i) + 1.0) / number_of_views, float(j + 1) / number_of_subwindows_per_view,
                         viewportNr[number_of_subwindows_per_view * i + j]);

      vis.setBackgroundColor(float(j * ((i % 2) / 10.0 + 1)) / number_of_subwindows_per_view,
                             float(j * ((i % 2) / 10.0 + 1)) / number_of_subwindows_per_view,
                             float(j * ((i % 2) / 10.0 + 1)) / number_of_subwindows_per_view,
                             viewportNr[number_of_subwindows_per_view * i + j]);

      vis.removeAllPointClouds(viewportNr[i * number_of_subwindows_per_view + j]);
      vis.removeAllShapes(viewportNr[i * number_of_subwindows_per_view + j]);
      std::stringstream window_id;
      window_id << "(" << i << ", " << j << ") ";
      if (title_subwindows.size() > j) {
        window_id << title_subwindows[j];
      }
      vis.addText(window_id.str(), 10, 10, window_id.str(), viewportNr[i * number_of_subwindows_per_view + j]);
    }
  }
  return viewportNr;
}

void PCLVisualizationParams::init(boost::program_options::options_description &desc, const std::string &section_name) {
  desc.add_options()((section_name + ".no_text").c_str(), po::value<bool>(&no_text_)->default_value(no_text_),
                     "optimizes visualization for paper (no text labels...)");
  desc.add_options()((section_name + ".vis_pt_size").c_str(),
                     po::value<int>(&vis_pt_size_)->default_value(vis_pt_size_),
                     "point size used for visualizing point clouds");
  desc.add_options()((section_name + ".fontsize").c_str(), po::value<int>(&fontsize_)->default_value(fontsize_),
                     "fontsize used for text in visualization");
  desc.add_options()((section_name + ".coordinate_axis_scale").c_str(),
                     po::value<float>(&coordinate_axis_scale_)->default_value(coordinate_axis_scale_),
                     "scale of coordinate axis in PCL visualization");
}
}  // namespace v4r
