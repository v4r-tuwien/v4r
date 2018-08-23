# Object Ground Truth Annotation Tool

This tool provides a Qt based Graphical User Interface for annotation of point clouds with 3D objects. The tool takes as input a set of point clouds to be annotated (argument `-i /path/to/pcd_files`) and a set of 3D object models (argument `-m /path/to/object/models`) and outputs for each cloud an annotation file with the name and 6DoF pose of each object annotated in the scene. For example the annotation of a PCD `cloud_0000000000.pcd` will write a file `cloud_0000000000_anno.txt` with an output like:
```
object_0 (0.4323): -0.0380448 -0.934844 -0.353033 1.75553 -0.890496 -0.128586 0.436463 0.400039 -0.453416 0.330975 -0.827578 1.20168 0 0 0 1
object_0 (0.46528): 0.891355 0.0517918 0.450352 0.975022 0.133585 0.919337 -0.370123 -0.0857727 -0.43319 0.390067 0.81253 1.38057 0 0 0 1
object_1 (0.278522): -0.731737 0.679881 -0.048327 1.56624 0.27005 0.224085 -0.936416 0.38826 -0.625818 -0.698255 -0.34757 1.33879 0 0 0 1
```
Each annotated object is one line starting with name of the object model, the approximate visibility of the object model with respect to the current viewpoint, and the annotated object pose also with respect to the current viewpoint. The visibility hereby computes the number of visible points taking into account model self-occlusion and occlusion by other points in the scene. It is defined as the ratio of visible points to the number of total points in the object model. The object pose is represented as 4x4 homogeneous transformation matrix aligning the object represented in the model coordinate system with the camera view. The coefficients of this matrix are stored in row-major order.

## Input arguments

* `-i /path/to/pcd_files`... Input path for test scenes. The tool will then assume that all  `.pcd` files within a folder belong to the same static scene. Each of these views will be registered by reading the `sensor_orienation` and `sensor_origin` field of the point cloud (If theses fields are set correctly, these point clouds are also visualized registered in the pcl viewer `pcl_viewer /path/to/pcd_files/*.pcd`). The advantage of using this method is that the individual views need to be annotated only once in the global coordinate system and the ground-truth can be projected to the individual views. Therefore, by clicking the button "Save ground truth", it will produce *n* output files, where *n* is the number of views of the scene.
* `-m /path/to/object/models`... Input path for the object models. It will search for 3D point clouds named `3D_model.pcd` and label them with the parent folder name.
* `--camera_parameters focal_length_x focal_length_y image_width image_height principal_point_x principal_point_y`... These are the camera parameters used to compute the occlusion of the individual object models.
* `-o /output/path`... path to where the annotation files will be saved to
* `[--rgbd_registration_mask file_path]`... This is an optional file path to a binary image with the same image size as the camera image size (`image_width x image_height`). It accounts for the fact that not 100% of the FOV is visible by both cameras after RGB-D registration. Each pixel of the `rgbd_registration_mask` indicates whether it is visible by both RGB and depth camera (white/255) or if it is outside the FOV for one of them (black/0). If this parameter is not defined, it will assume all pixel are valid (white).
* [`-a /path/to/pre-annotations`]... optional path to already existing `.anno` text files that are used to initialize the ground-truth anootation process. This could for instance be used if object recognition was already run on the scenes and could provide some initial "ground-truth". It assumes that the relative path and filename is the same as the input cloud.
* [`-h`]... additional parameters are shown with this argument


## Adding an object to the hypotheses list  
First, select the object you want to add by selecting it from the model list (`Available object models`) on the left side of the GUI and pressing the `add` button. This will place the object in the origin of the scene. Next, the object needs to be roughly aligned to the correct position by one of the following approaches:  
* Using the alignment buttons in the control box to move or rotate the object along the respective coordinate axis.
* By picking points (`SHIFT + click`)

### Point picking   
If an object model has been selected, it can be aligned to the scene by clicking points in the point cloud. A point can be picked by holding the `SHIFT` key and clicking a point in the model or scene view. If multiple points are picked (i.e. at least 3) from both model and scene, a transformation will be estimated using Singular Value Decomposition of all clicked points. The correspondences of model and scene points are defined by the order the points have been picked (e.g. first clicked point in the scene corresponds to first clicked points in the model,...). Picked points are highlighted with a unique color in the respective visualization window and correspondences have the same color.

*Note that due to VTK point picking issues, the GUI shows two windows for the object model. The left window is supposed for selection of the points while the right window only visualizes the already picked-points on the object model. Similarly, for the scene, the user is supposed to select the point in the lower scene windows while the already picked points are shown in the upper scene window.*

### Fine registration
The object can be fine registered to the scene either using again the alignment buttons in the control box or using the Iterative Closest Point (ICP) approach to align the object given in its roughly aligned pose to the scene.

## Removing an object from the annotation list
If you want to remove an object from the annotation, just select the object in the hypotheses list (`Objects in scene`) and click the button "remove".

## Saving the annotation
If you are happy with the annotation, you can store the annotated scene by clicking the button "Save ground truth". It will store the annotated file in the directory provided by argument `-o /path/to/output_dir` when calling the program.


## Keyboard events
There are some keyboard events for improved user interaction:
 * `b`... toggle background color
 * `c`... turn on/off color for scene visualization windows
 * `ARROW keys`... move selected object hypothesis along x/y

## Using a background cloud
The argument `--bg_cloud` allows to set a background cloud that is subtract from any given input cloud. This reduces the probability of accidentally picking a wrong point in the scene as well as it reduces the risk of ICP to catch points from the background. The subtraction is computed by creating a Kd-Tree of the background cloud and searching for neighboring points within a certain radius (`--subtraction_radius`).

## 2D visualization
Besides checking the current annotation in the 3D point cloud, the annotation can also be verified by showing a 2D projection of the annotated objects overlaid to the input scene. The 2D projection is hereby computed by depth buffering using the a given camera pinhole model

## Iterating through input files
After you are done with the annotation of a scene, save the annotation by clicking the button "Save ground truth" and then load the next scene by clicking the button `->`.
