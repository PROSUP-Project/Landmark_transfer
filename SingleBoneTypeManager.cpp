#include <itkMesh.h>
#include <itkStandardMeshRepresenter.h>
#include <itkStatisticalModel.h>

#include <boost/thread.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/bind.hpp>

#include <DataManager.h>
#include <StatisticalModel.h>

#include "SingleBoneTypeManager.hpp"
#include "FileIO.hpp"

SingleBoneTypeManager::SingleBoneTypeManager(){
	;
}

SingleBoneTypeManager::~SingleBoneTypeManager(){
	for (int i = 0; i < bones.size(); ++i) {
		delete bones[i];
	}
}
		
void SingleBoneTypeManager::addBone(Bone* b){
	if(b!=NULL){
		bones.push_back(b);
	}
}
		
std::vector<Bone*> SingleBoneTypeManager::getBones(){
	return bones;
}

Bone* SingleBoneTypeManager::get_longest_bone(){
	//if this manager manages no bones nothing can be done
	if(bones.size() < 1){
		return NULL;
	}
	Bone* b = bones[0];
	for (unsigned i = 1; i < bones.size() ; ++i) {
		if(b->getLength() < bones[i]->getLength()){
			b = bones[i];
		}
	}
	return b;
}

void SingleBoneTypeManager::align_bones(int rigit_iter){
	//if this manager manages no bones nothing can be done
	if(bones.size() < 1){
		return;
	}
	//the first bone will be the reference for the alignment
	Bone* reference = bones[0];
	//align all bones
	for (unsigned i = 0; i < bones.size() ; ++i) {
		bones[i]->align_with(reference, rigit_iter);
	}
	//free some memory
	for (unsigned i = 0; i < bones.size() ; ++i) {
		bones[i]->free_not_aligned_data();
	}
}

void SingleBoneTypeManager::align_bones(itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer model, int rigit_iter){
	//if this manager manages no bones nothing can be done
	if(bones.size() < 1){
		return;
	}
	std::string temp = bones[0]->getOutputFolder() + "/GaussRepresenterReference.vtk";
	saveVTKFile(model->GetRepresenter()->GetReference(), temp);
	vtkSmartPointer<vtkPolyData> vtkReference = loadVTKPolydata(temp);
	//align all bones
	for (unsigned i = 0; i < bones.size() ; ++i) {
		bones[i]->align_with(vtkReference, rigit_iter);
	}
	//free some memory
	for (unsigned i = 0; i < bones.size() ; ++i) {
		bones[i]->free_not_aligned_data();
	}
}

void SingleBoneTypeManager::multicore_align_bones(SingleBoneTypeManager* sbtm, int rigit_iter){
	sbtm->align_bones(rigit_iter);
}
		
itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer SingleBoneTypeManager::getGPModel(double sigma1, double sigma2, double relation, int scale, int num_comp){
	Bone* bone = get_longest_bone();
	if(bone == NULL){
		return NULL;
	}
	return bone->getGaussianProcessModel(sigma1, sigma2, relation, scale, num_comp);
}

itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer SingleBoneTypeManager::getMultiLevelGPModel(double sigma1, int scale, int levels, int num_comp){
	Bone* bone = get_longest_bone();
	if(bone == NULL){
		return NULL;
	}
	return bone->getMultiLevelGaussianProcessModel(sigma1, scale, levels, num_comp);
}

statismo::DataManager< itk::Mesh<float, 3> >* SingleBoneTypeManager::getDatamanager(){
	//if this manager manages no bones nothing can be done
	if(bones.size() < 1){
		return NULL;
	}
	//the reference will be the longest bone as it gives the best results
	Bone* reference = get_longest_bone();
	
	//Reference mesh for the statistical model
	itk::Mesh<float, 3>::Pointer aligned_reference_mesh = reference->get_aligned_data();
	reference->addLandmarksToMesh(aligned_reference_mesh);
	
	//Mesh representer for the statistical model
	itk::StandardMeshRepresenter<float, 3>::Pointer model_representer = itk::StandardMeshRepresenter<float, 3>::New();
	model_representer->SetReference(aligned_reference_mesh);
	
	//DataManager for all Meshes
	statismo::DataManager< itk::Mesh<float, 3> >* datamanager(statismo::DataManager< itk::Mesh<float, 3> >::Create(model_representer));
	
	return datamanager;
}

void SingleBoneTypeManager::addCorrespondentDataToDataManager(statismo::DataManager< itk::Mesh<float, 3> >* datamanager, itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer gaussModel, int gauss_fit_iter, float volume_percentage){
	//if this manager manages no bones nothing can be done
	if(bones.size() < 1){
		return;
	}
	if(datamanager == NULL){
		return;
	}
	std::vector<boost::thread*> corr_threads;
	for (unsigned i = 0; i < bones.size(); ++i) {
		corr_threads.push_back(new boost::thread(Bone::multicore_correspond, bones[i], datamanager, gaussModel, gauss_fit_iter, volume_percentage));
	}
	for(int i = 0; i < corr_threads.size(); ++i){
		corr_threads[i]->join();
		delete corr_threads[i];
	}
}

void SingleBoneTypeManager::addCorrespondentDataToDataManager(statismo::DataManager< itk::Mesh<float, 3> >* datamanager, itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer gaussModel, int gauss_fit_iter, float volume_percentage, double variance){
	//if this manager manages no bones nothing can be done
	if(bones.size() < 1){
		return;
	}
	if(datamanager == NULL){
		return;
	}
	if(variance < 0.01){
		variance = 0.01;
	}
	std::vector<boost::thread*> corr_threads;
	for (unsigned i = 0; i < bones.size(); ++i) {
		corr_threads.push_back(new boost::thread(Bone::multicore_correspond_with_constrains, bones[i], datamanager, gaussModel, gauss_fit_iter, volume_percentage, variance));
	}
	for(int i = 0; i < corr_threads.size(); ++i){
		corr_threads[i]->join();
		delete corr_threads[i];
	}
}
