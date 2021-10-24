### Intrinsic and Extrinsic Self Calibration of Roadside LiDAR and Camera

#### Method

![fig3](/media/jingxin/Windows-SSD/Users/jingxin/Desktop/fig3.jpg)

#### Requirements

- ＞=cmake 3.10
- pcl 1.8
- Eigen3
- Ceres 2.0.0
- OpenCV 3.4.5

#### Usage

- Extract features

  - set image path:

    modify the image_path paramter in config/param.yaml like:

    ```
    image_path:"/home/jingxin/calibration/lidar_camera_calibration/feature/image0/001001.jpg"
    ```

  - 2d features:

    ```bash
    python src/extract_feature.py
    ```

    When you run this program, the image will appear in a window. The windows may be small at first, you can adjust it by dragging it. Then you can extract 2d features by doing the following operations:

    ```
    left click: select/add a point
    mid click: delete a point
    right click: form a feature out of the previously selected points
    ```

    After right click, the 2d feature will be saved in the image_path folder.

    We always select line features and regions features. For line features, they are always formed by two points. For region features, they can be formed by many points, but since the opencv limitation, you have to select the points in clockwise or counterclockwise order, otherwise they can not form a closed region feature.

  - set feature num:

    modify the feature_num paramter in config/param.yaml like:

    ```
    feature_num: 10
    ```

  - 3d features:

    The demo map can be obtained by the link following:

     https://pan.baidu.com/s/1TsBk4bmP8vH122Mw0fKp0A code: j6ue 
    
    We use the CloudCompare software to crop corresponding 3d features from map point cloud. The software is installed in windows. The saved result from the software is always txt format. So we can use the program to transform them to pcd format:

    ```
    python txt2pcd.py
    ```

    Finally, the 3d_feature folder will have pcd files whose number is feature_num.
    
    After feature extraction, the folder will be like this:
    
    ```
    └── traffic-LiDAR-camera_calibration
           └── feature    
                  └── image0  <-- your camera folder
                         ├── 3d_features       <-- 3d features
                         		├── feature1.pcd
                         		├── feature1.txt 
                         		...
                         		├── feature10.pcd
                         		├── feature10.txt
                         ├── feature           <-- 2d features
                         		├── feature1.jpg
                         		...
                         		├── feature10.jpg
                         ├── feature_bit       <-- 2d bit featuress
                         		├── feature1.jpg
                         		...
                         		├── feature10.jpg
                         |── 001001.jpg        <-- your image file
    ```

- Run optimization

  - modify the initial intrinsic and extrinsic paramter in config/param.yaml like:

    ```
    # image0
    fx: 2000
    fy: 2000
    cx: 960
    cy: 540
    extrinsic: [9.88910941e-01,  1.48500658e-01,  1.64494210e-03,  2.20000000e+01,
                5.18266263e-02, -3.34706924e-01, -9.40895996e-01, -1.50000000e+01,
                -1.39173101e-01,  9.30547597e-01, -3.38691627e-01,  8.00000000e+01,
                0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]
    ```

  - build and run main.cpp

    ```
    cd traffic-LiDAR-camera_calibration
    mkdir build
    cd build
    cmake ..
    make
    ./main
    ```

- Results:

  After the convergence, you will see the results like this:

  ![image-20211024151254437](/home/jingxin/.config/Typora/typora-user-images/image-20211024151254437.png)

  The intrinsic and extrinsic results will be printed on the console. And the initial reproject and final reproject will be shown.

#### TODO

- [ ] rough calibration
- [ ] demo video