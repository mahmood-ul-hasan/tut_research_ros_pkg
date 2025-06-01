
    /////////////////////////////////////////////////////////////////////////////////////////
 if (add_first_time && !current_line_clouds.empty()) {
        std::cout << "------------------------------------------------ add_first_time " << std::endl;
        add_first_time = false;

        for (size_t j = 0; j < current_line_clouds.size(); ++j) {
            if (current_line_clouds[j]) {
                if (merged_planes.size() <= j) {
                    merged_planes.resize(j + 1);
                    pre_merged_planes_endpoints.resize(j + 1);
                }
                if (!merged_planes[j]) {
                    merged_planes[j] = PointCloudT::Ptr(new PointCloudT);
                }
                *merged_planes[j] = *current_line_clouds[j];
                pre_merged_planes_endpoints[j] = current_line_endpoints[j];
                std::cout << "-- Number of points in merged_planes [" << j << "]: " << merged_planes[j]->size() << std::endl;
            }
        }
        std::cout << "merged_planes.size() " << merged_planes.size() << std::endl;
        std::cout << "------------------------------------------------ add_first_time " << std::endl;
    } 
    
    else 
    
    {
        for (size_t i = 0; i < current_line_clouds.size(); ++i) {
            std::cout << "--------------------------------------------line_seg No " << i << std::endl;
            PointCloudT::Ptr current_line = current_line_clouds[i];
            auto current_endpoints = current_line_endpoints[i];
            std::cout << "-- Number of points in current_line_clouds[" << i << "]: " << current_line->size() << std::endl;

            bool merged = false;

                PointT p1 = current_endpoints.first;
                PointT p2 = current_endpoints.second;
    
                 std::vector<float> X1 = {p1.x, p2.x};
                 std::vector<float> Y1 = {p1.y, p2.y};
                 std::ostringstream label1;
                label1 << "current_endpoints (i = " << i << ")";
                plt::named_plot(label1.str(), X1, X1); // Red line for current_endpoints


            for (size_t j = 0; j < merged_planes.size(); ++j) {
              
                std::cout << "merged_plane No " << j << std::endl;
                PointCloudT::Ptr plane = merged_planes[j];
                auto merged_endpoints = pre_merged_planes_endpoints[j];
                std::cout << "-- Number of points in merged_planes [" << j << "]: " << plane->size() << std::endl;

              
                PointT p3 = merged_endpoints.first;
                PointT p4 = merged_endpoints.second;
                std::vector<float> X2 = {p3.x, p4.x};
                std::vector<float> Y2 = {p3.y, p4.y};
                std::ostringstream label2;
                label2 << "plane_endpoints (j = " << j << ")";
                plt::named_plot(label2.str(), X2, Y2, "b-"); // Blue line for plane_endpoints



                   // Function to compute the perpendicular distance from a point (px, py) to a line defined by (x1, y1) and (x2, y2)
    auto distance_point_to_line = [](float px, float py, float x1, float y1, float x2, float y2) {
        float A = y2 - y1;
        float B = x1 - x2;
        float C = x2 * y1 - x1 * y2;
        return std::fabs(A * px + B * py + C) / std::sqrt(A * A + B * B);
    };

  // Compute distances from each endpoint of line 1 to line 2
    float d1 = distance_point_to_line(p1.x, p1.y, p3.x, p3.y, p4.x, p4.y);
    float d2 = distance_point_to_line(p2.x, p2.y, p3.x, p3.y, p4.x, p4.y);

    // Compute distances from each endpoint of line 2 to line 1
    float d3 = distance_point_to_line(p3.x, p3.y, p1.x, p1.y, p2.x, p2.y);
    float d4 = distance_point_to_line(p4.x, p4.y, p1.x, p1.y, p2.x, p2.y);


                std::cout << "current_endpoints start " << p1.x << " , " << p1.y << " , " << p1.z << " end " << p2.x << " , " << p2.y << " , " << p2.z << std::endl;
                std::cout << "plane_endpoints start " << p3.x << " , " << p3.y << " , " << p3.z << " end " << p4.x << " , " << p4.y << " , " << p4.z << std::endl;

                // float d1 = std::sqrt(std::pow(p1.x - p3.x, 2) + std::pow(p1.y - p3.y, 2) + std::pow(p1.z - p3.z, 2));
                // float d2 = std::sqrt(std::pow(p2.x - p4.x, 2) + std::pow(p2.y - p4.y, 2) + std::pow(p2.z - p4.z, 2));

                std::cout << "distance_threshold " << distance_threshold << " , d1 " << d1 << " , d2 " << d2 <<  " , d3 " << d3 << " , d4 " << d4 <<std::endl;

                if (d1 < distance_threshold && d2 < distance_threshold && d3 < distance_threshold && d4 < distance_threshold) {
                    Eigen::Vector3f v1(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
                    Eigen::Vector3f v2(p4.x - p3.x, p4.y - p3.y, p4.z - p3.z);
                    v1.normalize();
                    v2.normalize();
                    float dot_product = v1.dot(v2);
                    float angle = std::acos(dot_product);
                    std::cout << "angle_threshold " << angle_threshold << " , std::abs(angle) " << std::abs(angle  * 180/ M_PI) << std::endl;

                    if (std::abs(angle * 180/ M_PI) < angle_threshold) {
                        *plane += *current_line;  // Merge the point clouds
                        pre_merged_planes_endpoints[j] = current_endpoints;
                        std::cout << "-- Number of points in combined_line_clouds[" << i << "]: " << plane->size() << std::endl;
                        merged = true;
                        break;
                    }
                }
            }

            if (!merged) {
                PointCloudT::Ptr new_plane(new PointCloudT);
                *new_plane = *current_line;
                merged_planes.push_back(new_plane);
                pre_merged_planes_endpoints.push_back(current_endpoints);
            }
        }
    }

    std::cout << "combined_line_clouds.size() " << merged_planes.size() << std::endl;

    // Add labels and show plot
    plt::xlabel("X axis");
    plt::ylabel("Y axis");
    plt::title("Line Plot of Endpoints");
    plt::legend();
    plt::show();
