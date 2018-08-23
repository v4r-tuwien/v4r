#include <glog/logging.h>
#include <v4r/apps/compute_recognition_rate.h>

// -m /media/Data/datasets/TUW/models/ -t /media/Data/datasets/TUW/validation_set/ -g
// /media/Data/datasets/TUW/annotations/ -r /home/thomas/recognition_results_eval/

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

  size_t tp, fp, fn;
  v4r::apps::RecognitionEvaluator e(e_param);
  e.compute_recognition_rate(tp, fp, fn);

  float precision = (float)tp / (tp + fp);
  float recall = (float)tp / (tp + fn);
  float fscore = 2 * precision * recall / (precision + recall);

  //            double score = 4*recall + precision - median_time_ms*0.01*0.01;    // we want to get as much hypotheses
  //            as possible - precision will be improved with verification

  std::cout << "tp: " << tp << ", fp: " << fp << ", fn: " << fn << ", precision: " << precision
            << ", recall: " << recall << ", fscore: " << fscore << std::endl;

  return 0;
}
