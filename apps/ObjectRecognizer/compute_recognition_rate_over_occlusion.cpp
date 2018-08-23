// -g /media/Data/datasets/TUW/annotations/ -r /home/thomas/recognition_results_eval/ -m
// /media/Data/datasets/TUW/models/

#include <glog/logging.h>
#include <v4r/apps/compute_recognition_rate.h>

namespace po = boost::program_options;

int main(int argc, char **argv) {
  v4r::apps::RecognitionEvaluator::Parameter e_param;

  google::InitGoogleLogging(argv[0]);
  po::options_description desc(
      "Evaluation of object recognition\n==========================================\nAllowed options:\n");
  desc.add_options()("help,h", "produce help message");
  e_param.init(desc);
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  if (vm.count("help")) {
    std::cout << desc << std::endl;
    return -1;
  }
  try {
    po::notify(vm);
  } catch (std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl << std::endl << desc;
    return -1;
  }

  v4r::apps::RecognitionEvaluator e(e_param);
  float recognition_rate = e.compute_recognition_rate_over_occlusion();
  (void)recognition_rate;  // silence unused variable warning
  return 0;
}
