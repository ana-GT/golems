#include <boost/algorithm/string.hpp>
#include <string>
#include <caffe/caffe.hpp>

//char* gSolver( "/home/ana/Software/caffe/models/YCB_svm/solver.prototxt" );
char* gSolver( "/home/ana/Software/caffe/models/YCB_svm/solver_thoughtful.prototxt" );
char* gWeights( "/home/ana/Software/caffe/models/VGG_ILSVRC_19_layers/VGG_ILSVRC_19_layers.caffemodel" );

// Load the weights from the specified caffemodel(s) into the train and
// test nets.
void CopyLayers(caffe::Solver<float>* solver, const std::string& model_list) {
  std::vector<std::string> model_names;
  boost::split(model_names, model_list, boost::is_any_of(",") );
  for (int i = 0; i < model_names.size(); ++i) {
    std::cout << "Copying weights from " << model_names[i]<< std::endl;
    solver->net()->CopyTrainedLayersFrom(model_names[i]);
    for (int j = 0; j < solver->test_nets().size(); ++j) {
      solver->test_nets()[j]->CopyTrainedLayersFrom(model_names[i]);
    }
  }
}



/****/
int main( int argc, char* argv[] ) {

  caffe::SolverParameter solver_param;
  printf("About to read solver \n");
  caffe::ReadSolverParamsFromTextFileOrDie( gSolver,
					    &solver_param );
  printf("Read solver...\n");

  // Set mode CPU
  caffe::Caffe::set_mode( caffe::Caffe::CPU );

  boost::shared_ptr<caffe::Solver<float> >
    solver(caffe::SolverRegistry<float>::CreateSolver(solver_param));
  printf("[DONE] Created solver \n");

  // Copy weights
  printf("-- Copying weights .. \n");
  CopyLayers( solver.get(), gWeights );
  printf("[DONE] Copied weights \n");	

  printf("-- Start optimization .. \n");
  solver->Solve();
  printf("[DONE] Solving is done \n");

}
