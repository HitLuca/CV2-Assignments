/*
 *  Author: Luca Simonetto
 *  SN: 11413522
 */

#include <iostream>
#include <stdlib.h>
#include <cmath>
#include <boost/format.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/impl/texture_mapping.hpp>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/marching_cubes_hoppe.h>

#include <eigen3/Eigen/Core>

#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/eigen.hpp>
    
#include "Frame3D/Frame3D.h"

#include <pcl/visualization/cloud_viewer.h>

// Struct containing color associations for one cloud point
struct multi_colored_point {
    std::vector<int> r;
    std::vector<int> g;
    std::vector<int> b;
    std::vector<int> intensity;
};

// Convert a depth image to a point cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr mat2IntegralPointCloud(const cv::Mat& depth_mat, const float focal_length, const float max_depth) {
    assert(depth_mat.type() == CV_16U);
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    const int half_width = depth_mat.cols / 2;
    const int half_height = depth_mat.rows / 2;
    const float inv_focal_length = 1.0 / focal_length;
    point_cloud->points.reserve(depth_mat.rows * depth_mat.cols);
    for (int y = 0; y < depth_mat.rows; y++) {
        for (int x = 0; x < depth_mat.cols; x++) {
            float z = depth_mat.at<ushort>(cv:: Point(x, y)) * 0.001;
            if (z < max_depth && z > 0) {
                point_cloud->points.emplace_back(static_cast<float>(x - half_width)  * z * inv_focal_length,
                                                 static_cast<float>(y - half_height) * z * inv_focal_length,
                                                 z);
            } else {
                point_cloud->points.emplace_back(x, y, NAN);
            }
        }
    }
    point_cloud->width = depth_mat.cols;
    point_cloud->height = depth_mat.rows;
    return point_cloud;
}

// Compute normals given a point cloud
pcl::PointCloud<pcl::PointNormal>::Ptr computeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*cloud_normals);
    pcl::copyPointCloud(*cloud, *cloud_normals);
    return cloud_normals;
}

// Transform a point cloud using a transformation matrix
pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const Eigen::Matrix4f& transform) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);
    return transformed_cloud;
}

// Transform a point cloud using a transformation matrix
template<class T>
typename pcl::PointCloud<T>::Ptr transformPointCloudNormals(typename pcl::PointCloud<T>::Ptr cloud, const Eigen::Matrix4f& transform) {
    typename pcl::PointCloud<T>::Ptr transformed_cloud(new typename pcl::PointCloud<T>());
    pcl::transformPointCloudWithNormals(*cloud, *transformed_cloud, transform);
    return transformed_cloud;
}


// ---IMPLEMENTATION---

// Merge the input frames into a single point cloud
pcl::PointCloud<pcl::PointNormal>::Ptr mergePointClouds(Frame3D frames[]) {
    double max_depth = 1;


    // empty point cloud
    pcl::PointCloud<pcl::PointNormal>::Ptr model_point_cloud(new pcl::PointCloud<pcl::PointNormal>());

    // for every frame
    for(int i=0; i<8; i++) {
        // extract the frame parameters
        cv::Mat depth_mat = frames[i].depth_image_;
        double focal_length = frames[i].focal_length_;
        Eigen::Matrix4f camera_pose = frames[i].getEigenTransform();
        
        // extract the frame cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud = mat2IntegralPointCloud(depth_mat, focal_length, max_depth);
        
        // compute the normals
        pcl::PointCloud<pcl::PointNormal>::Ptr point_cloud_with_normals = computeNormals(point_cloud);

        // transform the point cloud
        point_cloud_with_normals = transformPointCloudNormals<pcl::PointNormal>(point_cloud_with_normals, camera_pose);

        // remove NaN values from the cloud
        std::vector<int> indices;
        pcl::PointCloud<pcl::PointNormal>::Ptr new_point_cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>());
        pcl:removeNaNNormalsFromPointCloud(*point_cloud_with_normals, *new_point_cloud_with_normals, indices);

        // add the cloud to the final one
        *model_point_cloud += *new_point_cloud_with_normals;
    }
    return model_point_cloud;
}

// Color the points in the input cloud using a vector of color associations and a coloring method
void colorPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud, std::vector<multi_colored_point> multiple_colors, int method) {
    switch(method) {
        // variance display
        case 0: {
            // calculate the color variances for every point
            std::vector<float> variances(multiple_colors.size());

            for(int i=0; i<multiple_colors.size(); i++) {
                multi_colored_point p = multiple_colors.at(i);
                float mean = 0;
                float variance = 0;

                for(int j=0; j<p.intensity.size(); j++) {
                    mean += p.intensity.at(j);
                }

                mean /= p.intensity.size();

                for(int j=0; j<p.intensity.size(); j++) {
                    variance += pow(p.intensity.at(j) - mean, 2);
                }
                variance /= p.intensity.size()-1;
                variances.at(i) = variance;
            }

            // determine minimum and maximum value for the variances
            float min_variance = std::min_element(variances.begin(), variances.end()) - variances.begin();
            float max_variance = std::max_element(variances.begin(), variances.end()) - variances.begin();

            // apply variance color map
            for(int i=0; i<variances.size(); i++) {
                // transform the variance values into a 0-255 range
                float value = 255.0 / (max_variance - min_variance) * (variances.at(i) - max_variance) + 255;
                point_cloud -> points[i].r = 255;
                point_cloud -> points[i].g = value;
                point_cloud -> points[i].b = 255;
            }
            break;
        }
        // first color
        case 1: {
            // apply the first matched color for every point
            for(int i=0; i<multiple_colors.size(); i++) {
                multi_colored_point p = multiple_colors.at(i);
                if(p.intensity.size() > 0){
                    point_cloud -> points[i].r = p.r.at(0);
                    point_cloud -> points[i].g = p.g.at(0);
                    point_cloud -> points[i].b = p.b.at(0);
                }
            }
            break;
        }
        // uncolored points
        case 2: {
            // color only the points that don't have any matches (generated by watertighting)
            for(int i=0; i<multiple_colors.size(); i++) {
                multi_colored_point p = multiple_colors.at(i);
                if(p.intensity.size() == 0){
                    point_cloud -> points[i].r = 255;
                    point_cloud -> points[i].g = 255;
                    point_cloud -> points[i].b = 255;
                } else {
                    point_cloud -> points[i].r = 128;
                    point_cloud -> points[i].g = 128;
                    point_cloud -> points[i].b = 128;
                }
            }
            break;
        }
        // average color
        case 3: {
            // color every point with the average value for each channel
            for(int i=0; i<multiple_colors.size(); i++) {
                multi_colored_point p = multiple_colors.at(i);
                if(p.intensity.size() > 0){
                    float tot_r = 0;
                    float tot_g = 0;
                    float tot_b = 0;
                    for(int j=0; j<p.intensity.size(); j++) {
                        tot_r += p.r.at(j);
                        tot_g += p.g.at(j);
                        tot_b += p.b.at(j);
                    }

                    point_cloud -> points[i].r = tot_r / p.intensity.size();
                    point_cloud -> points[i].g = tot_g / p.intensity.size();
                    point_cloud -> points[i].b = tot_b / p.intensity.size();
                }
            }
            break;
        }
        // average intensity
        case 4: {
            // for every point
            for(int i=0; i<multiple_colors.size(); i++) {
                // calculate the mean intensity
                multi_colored_point p = multiple_colors.at(i);
                if(p.intensity.size() > 0){
                    float mean_i = 0;
                    for(int j=0; j<p.intensity.size(); j++) {
                        mean_i += p.intensity.at(j);
                    }

                    mean_i /= p.intensity.size();

                    // find the intensity closest to the mean value
                    int nearest_value = 0;
                    int nearest_index = 0;

                    for(int j=0; j<p.intensity.size(); j++) {
                        if(abs(mean_i - p.intensity.at(j)) < abs(mean_i - nearest_value)) {
                            nearest_value = p.intensity.at(j);
                            nearest_index = j;
                        }
                    }
                    point_cloud -> points[i].r = p.r.at(nearest_index);
                    point_cloud -> points[i].g = p.g.at(nearest_index);
                    point_cloud -> points[i].b = p.b.at(nearest_index);
                }
            }
            break;
        }
        // gray color
        case 5: {
            // apply gray color to every point (for displaying the initial mesh)
            for(int i=0; i<multiple_colors.size(); i++) {
                multi_colored_point p = multiple_colors.at(i);
                point_cloud -> points[i].r = 128;
                point_cloud -> points[i].g = 128;
                point_cloud -> points[i].b = 128;
            }
            break;
        }
        // median intensity
        case 6: {
            // for every point
            for(int i=0; i<multiple_colors.size(); i++) {
                multi_colored_point p = multiple_colors.at(i);
                if(p.intensity.size() > 0) {
                    // if there are only two intensities use the color with lowest intensity
                    if (p.intensity.size() == 2) {
                        int min = 255*3;
                        int index = -1;
                        for(int j=0; j<p.intensity.size(); j++) {
                           if(p.intensity.at(j) < min) {
                               index = j;
                           }
                        }
                        point_cloud -> points[i].r = p.r.at(index);
                        point_cloud -> points[i].g = p.g.at(index);
                        point_cloud -> points[i].b = p.b.at(index);
                    // if there is 1 or more than 2 intensities
                    } else {
                        // sort the intensity vector
                        std::vector<int> sorted_i = p.intensity;
                        std::sort(sorted_i.begin(), sorted_i.end());

                        // extract the median intensity and apply the color
                        int median_i = sorted_i.at(round((sorted_i.size() -1) / 2.0));
                        int index = -1;
                        for(int j=0; j<p.intensity.size(); j++) {
                           if(p.intensity.at(j) == median_i) {
                               index = j;
                               break;
                           }
                        }
                        point_cloud -> points[i].r = p.r.at(index);
                        point_cloud -> points[i].g = p.g.at(index);
                        point_cloud -> points[i].b = p.b.at(index);
                    }
                }
            }
            break;
        }
    }
}


// Texture the input mesh, given original frames and coloring method
void texture(pcl::PolygonMesh &mesh, Frame3D frames[], int coloring_method) {
    // extract polygons and point cloud from the mesh
    std::vector<pcl::Vertices> polygons = mesh.polygons;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromPCLPointCloud2(mesh.cloud, *point_cloud);

    // initialize the color associations vector
    std::vector<multi_colored_point> multiple_colors(point_cloud->size());

    // for every frame
    for(int i=0; i<8; i++) {
        // extract the parameters
        cv::Mat depth_mat = frames[i].depth_image_;
        cv::Mat rgb_image = frames[i].rgb_image_;
        double focal_length = frames[i].focal_length_;
        Eigen::Matrix4f camera_pose = frames[i].getEigenTransform();

        // transform the input cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_point_cloud = transformPointCloud(point_cloud, camera_pose.inverse());

        // bool vector to store if a point has been already considered
        std::vector<bool> already_checked(point_cloud->size());

        // for every polygon
        for (int j=0; j<polygons.size(); j++) {
            // calculate the normal vector
            pcl::Vertices polygon = polygons[j];
            int idx0 = polygon.vertices[0];
            int idx1 = polygon.vertices[1];
            int idx2 = polygon.vertices[2];

            pcl::PointXYZRGB pt0 = transformed_point_cloud -> points[idx0];
            pcl::PointXYZRGB pt1 = transformed_point_cloud -> points[idx1];
            pcl::PointXYZRGB pt2 = transformed_point_cloud -> points[idx2];

            Eigen::Vector3f vec12(pt1.x-pt0.x,pt1.y-pt0.y,pt1.z-pt0.z);
            Eigen::Vector3f vec23(pt2.x-pt1.x,pt2.y-pt1.y,pt2.z-pt1.z);
            Eigen::Vector3f vecNorm = vec12.cross(vec23);
            vecNorm.normalize();

            // if the polygon is pointing towards the camera
            if(vecNorm[2] < -0.3) {
                // apply algorithm to extract x and y coords
                int cx = depth_mat.cols / 2;
                int cy = depth_mat.rows / 2;
                for(int idx=0; idx<polygon.vertices.size(); idx++) {
                    int cloud_index = polygon.vertices[idx];

                    // continue only if it's a new point
                    if(!already_checked.at(cloud_index)) {
                        pcl::PointXYZRGB original_point = point_cloud -> points[cloud_index];
                        pcl::PointXYZRGB transformed_point = transformed_point_cloud -> points[cloud_index];

                        int u_unscaled = std::round(focal_length * (transformed_point.x / transformed_point.z) + cx);
                        int v_unscaled = std::round(focal_length * (transformed_point.y / transformed_point.z) + cy);

                        float u = static_cast<float>(u_unscaled / (float)depth_mat.cols);
                        float v = static_cast<float>(v_unscaled / (float)depth_mat.rows);

                        // if both u and v are acceptable
                        if(u>=0 && u<1 && v>=0 && v<1) {
                            int x = std::floor(rgb_image.cols * u);
                            int y = std::floor(rgb_image.rows * v);

                            // extract bgr vector from color map
                            cv::Vec3b bgr = rgb_image.at<cv::Vec3b>(y, x);

                            // add entry to the color associations vector
                            multiple_colors.at(cloud_index).r.push_back(bgr.val[2]);
                            multiple_colors.at(cloud_index).g.push_back(bgr.val[1]);
                            multiple_colors.at(cloud_index).b.push_back(bgr.val[0]);
                            multiple_colors.at(cloud_index).intensity.push_back(bgr.val[0]+bgr.val[1]+bgr.val[2]);

                            // set this point as checked
                            already_checked.at(cloud_index) = true;
                        }
                    }
                }
            }
        }
    }

    // color the points using the associations vector
    colorPoints(point_cloud, multiple_colors, coloring_method);

    // put the new cloud back into the mesh
    pcl::toPCLPointCloud2(*point_cloud, mesh.cloud);
}


// Display the point cloud contained in the mesh
void displayPointCloud(pcl::PolygonMesh mesh) {
    // extract the point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromPCLPointCloud2(mesh.cloud, *point_cloud);

    // initialize and start viewer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (1, 1, 1);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (point_cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

// Display the complete input mesh
void displayMesh(pcl::PolygonMesh mesh) {
    // initialize and start viewer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(1, 1, 1);
    viewer->addPolygonMesh(mesh, "meshes", 0);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

// Generate the Poisson mesh of the input cloud
pcl::PolygonMesh::Ptr poissonMesh(pcl::PointCloud<pcl::PointNormal>::Ptr &merged_frames) {
    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setInputCloud(merged_frames);
    pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
    poisson.reconstruct(*mesh);
    return mesh;
}

// Generate the marching cubes mesh of the input cloud
pcl::PolygonMesh::Ptr marchingCubesMesh(pcl::PointCloud<pcl::PointNormal>::Ptr &merged_frames) {
    // initialize search tree
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
    tree->setInputCloud(merged_frames);

    // initialize marching cubes algorithm
    pcl::MarchingCubesHoppe<pcl::PointNormal> *mc = new pcl::MarchingCubesHoppe<pcl::PointNormal>();
    pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
    mc->setInputCloud(merged_frames);
    mc->setSearchMethod(tree);
    // denser grid size
    mc->setGridResolution(100, 100, 100);
    // very low iso level
    mc->setIsoLevel(0.005);
    mc->reconstruct(*mesh);
    return mesh;
}

// Main function
int main(int argc, char *argv[]) {
    // coloring method choice
    // 0-variance display
    // 1-first color
    // 2-uncolored points
    // 3-average channel
    // 4-average intensity
    // 5-gray color
    // 6-median intensity
    int coloring_method = 6;

    if (argc != 2)
        return 0;
    
    Frame3D frames[8];
    
    for (int i = 0; i < 8; ++i) {
        frames[i].load(boost::str(boost::format("%s/%05d.3df") % argv[1] % i));
    }

    cout << "Loaded frames" << endl;

    // merge the input frames
    pcl::PointCloud<pcl::PointNormal>::Ptr merged_frames = mergePointClouds(frames);

    cout << "Merged point clouds" << endl;

    // generate the mesh using one of the two methods
    //pcl::PolygonMesh::Ptr mesh = marchingCubesMesh(merged_frames);
    pcl::PolygonMesh::Ptr mesh = poissonMesh(merged_frames);

    cout << "Constructed mesh" << endl;

    // texture the mesh
    texture(*mesh, frames, coloring_method);

    cout << "Textured model" << endl;

    // display the mesh (or the cloud)
    //displayPointCloud(*mesh);
    displayMesh(*mesh);

    return 0;
}
