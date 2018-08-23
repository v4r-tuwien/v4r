# Multi-modal RGB-D Object Instance Recognizer

In this tutorial you will learn how to detect objects in 2.5 RGB-D point clouds. The proposed multi-pipeline recognizer will detect 3D object models and estimate their 6DoF pose in the camera's field of view.


## Object Model Database  
The model folder structure is structured as:

```
object-model-database  
│
└───object-name-1
│   │    [3D_model.pcd]
│   │
│   └─── views
│        │   cloud_000000.pcd
│        │   cloud_00000x.pcd
│        │   ...
│        │   object_indices_000000.txt
│        │   object_indices_00000x.txt
│        │   ...
│        │   pose_000000.txt
│        │   pose_00000x.txt
│        │   ...
│   │
│   └─── [trained-pipeline-1]*
│        │   ...
│   │
│   └─── [trained-pipeline-x]*
│        │   ...
│   
└───object-name-2
│   │    [3D_model.pcd]
│   │
│   └─── views
│        │   cloud_000000.pcd
│        │   cloud_00000x.pcd
│        │   ...
│        │   object_indices_000000.txt
│        │   object_indices_00000x.txt
│        │   ...
│        │   pose_000000.txt
│        │   pose_00000x.txt
│        │   ...
│   │
│   └─── [trained-pipeline-1]*
│        │   ...
│   │
│   └─── [trained-pipeline-x]*
│        │   ...
│        │   ...
│ ...
```
`* this data / folder will be generated`

Objects are trained from several training views. Each training view is represented as
 * an organized point cloud (`cloud_xyz.pcd`),
  * the segmentation mask of the object (`object indices_xyz.txt`) ,
  * and a camera pose ( `pose_xyz.pcd`) that aligns the point cloud into a common coordinate system when multiplied by the given 4x4 homogenous transform.

The training views for each object are stored inside a `view`folder that is part of the object model folder. The object model folder is named after the object and contains all information about the object, i.e. initially contains the `views` and a complete 3D model ( `3D_model.pcd` ) of the object that is in the same coordinate system as the one in views. The complete model is used for hypotheses verification and visualization purposes.

At the first run of the recognizer (or whenever retraining is desired), the object model will be trained with the features of the selected pipeline. The training might take some time and extracted features and keypoints are stored in a subfolder called after its feature descriptor (e.g. sift keypoints for object-name-1 will be stored in `model-database/object-name-1/sift/keypoints.pcd`). If these files already exist, they will be loaded directly from disk and skip the training stage. Please note that the trained directory needs to be deleted whenever you update your object model (e.g. by adding/removing training views), or the argument `--retrain` set when calling the program.

If you have not obtained appropriate data yet, you can download example model files together with test clouds  by running
```
./scripts/get_TUW.sh
```
from your v4r root directory. The files (2.43GB) will be extracted into `data/TUW`.

## Usage
Assuming you built the ObjectRecognizer app, you can now run the recognizer. If you run it for the first time, it will automatically extract the descriptors of choice from your model data (`-m`).
The test directory can be specified by the argument `-t`. The program accepts either a specific file or a folder with test clouds. To use 2D features, these test clouds need to be organized.

Example command:

```
./build/bin/ObjectRecognizer -m data/TUW/TUW_models -t data/TUW/test_set --cfg apps/ObjectRecognizer/cfg/object_reco_config.ini -z 2.5 -v
```

## Parameters:
 Parameters for the recognition pipelines and the hypotheses verification are loaded from the config .ini file by default and can be overwritten by command line arguments. To list available parameters, call the recognizer with the additional flag `-h`.


If you want to visualize your results, add argument `-v`. Note that this is blocking and you have to close the window or press "e" on your keyboard to continue.

To get debug information printed on your console, add argument `--verbosity 2`. To disable verbose output, add `--verbosity -1`.

You can restrict the search space for objects with a couple of parameters. This will greatly reduce the runtime for object recognition:
Distance filter:
you can reduce the search space to everything within some distance max_object_distance by setting the additional flag `--or_multipipeline.chop_z 1.5`  (This value is the distance in meter in z-direction of the camera until objects are searched for)

### Plane removal:
You can remove dominant planes from your search space by adding the additional flag `--or_multipipeline.remove_planes 1`
This will search for dominant planes such as floor or desks first, and remove those planes (and everything below) if they have at least `--or_multipipeline.min_plane_inliers 20000` points. Also depending on your resolution and how far away you are from the plane, you might want to change `--or_multipipeline.min_plane_inliers` (increase for larger resolution and closer views, decrease for smaller resolution and distant views). Also this might not work well for cluttered scenes - so you might want to set  `--or_multipipeline.remove_planes 0`

###Feature selection.
Depending on the objects, you want to search for, you might want to enable/disable certain types of feature detectors. Currently V4R uses two different types of local feature detectors - one that searches for visual 2D texture information and one that searches for distinctive 3D geometric traits. They can be either used independently or in parallel
`--or_multipipeline.do_local_2d 1`
This will enable the visual 2D feature detector. This works especially well for textured objects. But note that good RGB-D camera registration is required for this.
`--or_multipipeline.do_shot 1`
This will enable the local 3D geometry feature detector (SHOT). Due to the noise of the depth camera, this might not work as well as the 2D feature detector.

Currently, we use BRISK features for 2D. To try SIFT (patented - not for commercial use!!), you can for example use `--or_multipipeline.local_2D_feature_estimator CVSIFT` and `--or_multipipeline.local_2D_keypoint_detector CVSIFT`. Note that depending on the chosen feature type, you might also need to change the distance metric used for feature matching. Binary features such as BRISK require HAMMING distance. For other descriptors (e.g. SIFT), you can use L2 distance by `--local_2d_pipeline.distance_metric L2`.

For 3D, we use SHOT with uniform sampling. To handle both small and medium-sized details, we use multiple support radii to describe a keypoint. Smaller geometric features are covered by `--or_multipipeline.keypoint_support_radii 0.04`, larger ones by `--or_multipipeline.keypoint_support_radii 0.08`. Every point that lies within a sphere of this radius in meter will be considered when computing the local 3D feature. If you only work with small or large objects, you might want to only use one support radius (decreases runtime). You can do this by simply removing the corresponding line in the config.ini file.

### Feature matching and clustering:
Feature matching is done via k-Nearest Neighbor Search for each feature detector. The number `--local_Xd_pipeline.knn` hereby defines the number of features to match. A small number will increase precision, while a larger number will increase recall and runtime. Note that we use a smaller number for 3D than for the 2D pipeline in the default config, as usually 2D descriptors are more reliable.

For clustering we require at least `--cg.size_thresh 3 `geometric consistent keypoint matches to generate an object hypotheses. A large number will increase precision, while a small number will increase recall and runtime.

Pose refinement is done via ICP. Your ICP settings will influence pose accuracy, runtime and even detection results (in case pose is not accurate enough, object might get rejected). Please adjust, `--hv.icp_iterations 10` (more iterations, means more accurate but slower) and `--icp_max_correspondence 0.02` (search space for nearest neighbor points in ICP. Depends on objects, distance and clutter) accordingly.

The resolution used to check hypotheses is set via `--hv.resolution_mm 5`. A rougher resolution will increase runtime but might decrease detection rate.

#### Global pipeline
If your scene configuration allows objects to be segmented, you can also consider using the global recognition pipeline which classifies the object as a whole. In the default configuration, first planes are removed (must be enabled, see above), and then remaining points are clustered based on Euclidean distance. Each cluster is then segmented by the specified classifer(s). Our generic framework, allows multiple classifiers to be used for classifying each cluster (generating multiple hypotheses). Let us give an example for for a single classifier (which should be sufficient for many cases): *TODO: FIX THESE DEPRECATED PARAMETERS*
```
	<global_recognition_pipeline_config>
		<count>1</count>
		<item_version>0</item_version>
		<item>cfg/global_config.xml</item>
:
:
	<global_feature_types_>
		<count>1</count>
		<item_version>0</item_version>
		<item>3232</item>
:
:
	<classification_methods_>
		<count>1</count>
		<item_version>0</item_version>
		<item>2</item>
```
The first group (global_recognition_pipeline_config) specifies the config XML file used for additional parameter settings for the global pipeline.

The second group  (global_feature_types_) specifies the **type of feature descriptrion**. Currently, following features are available (for a full and updated list of available feature types, please double check the header file `modules/features/include/v4r/features/types.h`):
 * ESF (type: 32)
 * Alexnet (type: 128) -- requires [Caffe library](https://github.com/BVLC/caffe)
 * Simple Shape (type: 1024)
 * Color (type: 2048)

Using an early fusion technique, feature vectors can be concatenated (just by adding the type numbers). In the example above, each cluster is described by concatenating the feature vectors from ESF, Alexnet, Simple Shape and Color into a single feature vector.

The third group (classification_methods_) specififes the **classification method** as defined in `modules/ml/include/v4r/ml/types.h`. Currently, following classification methods are available:
 * Nearest Neighbor (type: 1)
 * SVM (type: 2)

 If you selected SVM as your classification method, please also set **SVM specific command line arguments**:
  * `--svm.do_scaling 1` (enables scaling of feature attributes to normalize range between 0 and 1)
  * `--svm.do_cross_validation 3` (does 3-fold cross validation on the SVM parameters)
  * `--svm.kernel_type 0` (0... linear kernel, 2... RBF kernel) - to speed up training, use linear for high-dimensional features
  *  `--svm.filename model.svm` (in case you have already trained your SVM, you can also just load the SVM from file)
  * `--svm.probability 1` (enables probability estimates)

Each item specifies the type of classifer used.

### Multiview Recognizer
To enable multiview recognition, use `--or_multipipeline.use_multiview 1`.

Additionally, you can also enable
```
or_multipipeline.use_multiview_hv 1
or_multipipeline.use_multiview_with_kp_correspondence_transfer 1
```
In most settings, we however recommend to leave `--or_multipipeline.use_multiview_with_kp_correspondence_transfer` disabled to save computation time.

The object recognizer will then treat all `*.pcd` files within a folder as observations belonging to the same multi-view sequence. An additional requirement of the multi-view recognizer is that all observations need to be aligned into a common reference frame. Therefore, each `*.pcd`file needs to be associated with a relative camera pose (coarse point cloud registration is not done inside the recognizer!!). We use the header fields `sensor_origin` and `sensor_orientation` within the point cloud structure to read this pose. The registration can so be checked by viewing all clouds with the PCL tool `pcd_viewer /path/to/my/mv_observations/*.pcd`.
Please check if the registration of your clouds is correct before using the multi-view recognizer!

### Hypotheses Verification (HV)
Generated object hypotheses are verified by the Hypotheses Verification framework which tries to find the subset of generated hypotheses that best explain the input scene in terms of detected objects avoiding potential redundancy.

For each object hypothesis, the HV stage first computes the visible object points under the current viewpoint. Objects that are less visible than the defined **minimum visible ratio**
`--hv.min_visible_ratio 0.05`
will be rejected (in the example above at least 5% of the object need to be visible).

Based on the visible object, it then runs ICP to refine the pose of the object. The ICP parameters are
```
hv.icp_iterations 30
hv.icp_max_correspondence 0.02
```

Next, a fitness (or confidence) score is computed for each object hypothesis. Hypotheses with a score lower than the **minimum fitness threshold**
`--hv.min_fitness 0.3`
will be rejected. You can adjust this value to allow weak hypotheses to be removed more easily (0... disables individual rejection, 1... rejects all hypotheses).

The score is computed based on the fit of the object hypothesis to the scene with respect to
 * Euclidean distance in XYZ
 * surface normal orientation
 * color in CIELAB color space

The importance of these terms, can be set by these arguments
```
	hv.inlier_threshold_xyz 0.01
	hv.inlier_threshold_normals_dotp 0.99
	hv.inlier_threshold_color 20
	hv.sigma_xyz 0.003
	hv.sigma_normals 0.05
	hv.sigma_color 10
	hv.w_xyz 0.25
	hv.w_normals 0.25
	hv.w_color 0.5
```


## Visualization

To **visualize** the results, use command line argument `-v`.

To **visualize intermediate hypotheses verification** results, you can use `--hv_vis_model_cues` and `--hv_vis_cues`.


## Output
Recognized results will be stored in a single text file in the folder defined by command line argument `-o`. Each detected object is one line starting with name (same as folder name) of the found object model followed by the confidence (between 0 for poor object hypotheses and 1 for very high confidence -- value in brackets), and the object pose as a 4x4 homogenous transformation matrix in row-major order aligning the object represented in the model coordinate system with the current camera view. Example output:
```
object_08 (0.251965): 0.508105 -0.243221 0.826241 0.176167 -0.363111 -0.930372 -0.0505756 0.0303915 0.781012 -0.274319 -0.561043 1.0472 0 0 0 1
object_10 (0.109282): 0.509662 0.196173 0.837712 0.197087 0.388411 -0.921257 -0.0205712 -0.171647 0.767712 0.335861 -0.545726 1.07244 0 0 0 1
object_29 (0.616981): -0.544767 -0.148158 0.825396 0.0723312 -0.332103 0.941911 -0.0501179 0.0478761 -0.770024 -0.301419 -0.562326 0.906379 0 0 0 1
object_34 (0.565967): 0.22115 0.501125 0.83664 0.0674237 0.947448 -0.313743 -0.0625169 -0.245826 0.231161 0.806498 -0.544174 0.900966 0 0 0 1
object_35 (0.60515): 0.494968 0.0565292 0.86707 0.105458 0.160923 -0.986582 -0.0275425 -0.104025 0.85388 0.153165 -0.497424 0.954036 0 0 0 1
object_35 (0.589806): -0.196294 -0.374459 0.906228 0.1488 -0.787666 0.610659 0.0817152 -0.331075 -0.583996 -0.697765 -0.414817 1.01101 0 0 0 1
```
To visualize results, add argument `-v`. This will visualize the input scene, the generated hypotheses and the verified ones (from bottom to top).   
For further parameter information, call the program with `-h` or have a look at the doxygen documents.  

## Evaluation
To evaluate the results, you can use the `compute_recognition_rate` program.

Example input:
```
./build/bin/ObjectRecognizer/compute_recognition_rate  -r /my/results/ -g /my_dataset/annotations/ -t /my_dataset/test_set -m /my_dataset/models --occlusion_thresh 0.95 --rot_thresh 45
```


## References
* https://www.acin.tuwien.ac.at/en/vision-for-robotics/software-tools/tuw-object-instance-recognition-dataset/

### Single-View Recognition
* Thomas Fäulhammer, Michael Zillich, Johann Prankl, Markus Vincze,
*A Multi-Modal RGB-D Object Recognizer*,
IAPR International Conf. on Pattern Recognition (ICPR), Cancun, Mexico, 2016

### Multi-View Recognition
* Thomas Fäulhammer, Michael Zillich and Markus Vincze
*Multi-View Hypotheses Transfer for Enhanced Object Recognition in Clutter,*
IAPR International Conference on Machine Vision Applications (MVA), Tokyo, Japan, 2015

or, if used with keypoint correspondence transfer:

* Thomas Fäulhammer, Aitor Aldoma, Michael Zillich and Markus Vincze,
*Temporal Integration of Feature Correspondences For Enhanced Recognition in Cluttered And Dynamic Environments,*
IEEE International Conference on Robotics and Automation (ICRA), Seattle, WA, USA, 2015
