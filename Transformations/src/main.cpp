#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <string>  
// add some other header files you need

constexpr double MY_PI = 3.1415926;
enum label {translate, scale, rotation, view, projection, three_d_pts_and_inds, skip};

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    // world -> camera coordinate transformation
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    // why multiple identity matrix?????
    view = view * translate;
    // std::clog << "view" << std::endl << view << std::endl;  // check data

    return view;
}


Eigen::Matrix4f get_model_matrix(float rotation_angle, Eigen::Vector3f T, Eigen::Vector3f S, Eigen::Vector3f P0, Eigen::Vector3f P1)
{
    // rotational_angle in radian
    // model coordinate -> world coordinate
    Eigen::Matrix4f M_trans, S_trans, R_trans;
    //Step 1: Build the Translation Matrix M_trans: (T(0)->t_x, T(1)->t_y, T(2)->t_z)
    M_trans << 1, 0, 0, T(0),
               0, 1, 0, T(1),
               0, 0, 1, T(2),
               0, 0, 0, 1;
    //Step 2: Build the Scale Matrix S_trans:
    S_trans << S(0), 0, 0, 0,
               0, S(1), 0, 0,
               0, 0, S(2), 0,
               0, 0, 0, 1;
    //Step 3: Implement Rodrigues' Rotation Formular, rotation by angle theta around axix u, then get the model matrix
	// The axis u is determined by two points, u = P1-P0: Eigen::Vector3f P0 ,Eigen::Vector3f P1  
    // Create the model matrix for rotating the triangle around a given axis. // Hint: normalize axis first
    Eigen::Vector3f rot_axis_norm = (P1-P0)/((P1-P0).norm());
    double ux=rot_axis_norm(0), uy=rot_axis_norm(1), uz=rot_axis_norm(2), costhe=cos(rotation_angle * M_PI / 180.0), sinthe=sin(rotation_angle * M_PI / 180.0);
    R_trans << costhe+(1-costhe)*(ux*ux), uy*ux*(1-costhe)-uz*sinthe, uz*ux*(1-costhe)+uy*sinthe, 0,
               uy*ux*(1-costhe)+uz*sinthe, costhe+(uy*uy)*(1-costhe), uz*uy*(1-costhe)-ux*sinthe, 0,
               uz*ux*(1-costhe)-uy*sinthe, uz*uy*(1-costhe)+ux*sinthe, costhe+(uz*uz)*(1-costhe), 0,
               0, 0, 0, 1;

	//Step 4: Use Eigen's "AngleAxisf" to verify your Rotation
	Eigen::AngleAxisf rotation_vector((rotation_angle * M_PI / 180.0), Eigen::Vector3f(ux, uy, uz));  
	Eigen::Matrix3f rotation_matrix = rotation_vector.toRotationMatrix();
    Eigen::Matrix4f rot;
    // rot = rotation_matrix;
    for (int i = 0; i <=2; i++) {
        for (int j = 0; j <= 2; j++) {
            rot(i, j) = rotation_matrix(i, j);
        }
    }
    for (int i = 0; i <= 2; i++) {
        rot(i, 3) = 0;
        rot(3, i) = 0;
    }
    rot(3,3) = 1;

    std::cout << "rotational matrix in Rodge caled: \n" << R_trans << std::endl;
    std::cout << "rotational matrix in AngleAxisf: \n" << rot << std::endl;

    Eigen::Matrix4f model = rot * S_trans * M_trans;

	return model;
}

void init_map(std::map<std::string, label>& map) {
    map["translate"] = translate;
    map["scale"] = scale;
    map["rotation"] = rotation;
    map["view"] = view;
    map["projection"] = projection;
    map["three_d_pts_and_inds"] = three_d_pts_and_inds;
}

void parse_ini_params(Eigen::Vector3f& T, Eigen::Vector3f& S, Eigen::Vector3f& P0, 
                Eigen::Vector3f& P1, Eigen::Vector3f& eye_pos, float& eye_fov, float& aspect_ratio, 
                float& zNear, float& zFar, std::vector<Eigen::Vector3f>& pos, std::vector<Eigen::Vector3i>& ind,
                float& angle) 
{
    std::ifstream ini_handler;
    std::string line, param, pt;
    int comma_one_ind, comma_two_ind;
    std::map<std::string, label> label_parser;
    label flag = skip;

    // initialize parsing map
    init_map(label_parser);

    // File Open in the Read Mode
    ini_handler.open("./config.ini");

    if (ini_handler.is_open()) {
        // Keep reading the file
        while(getline(ini_handler, line))
        {
            // parse label
            if (line.substr(0,1).compare("[") == 0) {
                flag = label_parser[line.substr(1, line.length()-2)];
                continue;
            }
            
            param = line.substr(0, line.find("="));

            // variable init
            switch (flag) {
                case translate: {

                    if (param.compare("tx") == 0) {
                        T(0) = std::stof(line.substr(line.find("=")+1));
                    }
                    else if (param.compare("ty") == 0) {
                        T(1) = std::stof(line.substr(line.find("=")+1));
                    }
                    else if (param.compare("tz") == 0) {
                        T(2) = std::stof(line.substr(line.find("=")+1));
                    }
                    
                    break;
                }
                case scale: {

                    if (param.compare("sx") == 0) {
                        S(0) = std::stof(line.substr(line.find("=")+1));
                    }
                    else if (param.compare("sy") == 0) {
                        S(1) = std::stof(line.substr(line.find("=")+1));
                    }
                    else if (param.compare("sz") == 0) {
                        S(2) = std::stof(line.substr(line.find("=")+1));
                    }
                    
                    break;
                }   
                case rotation: {
                        
                    if (param.compare("p0x") == 0) {
                        P0(0) = std::stof(line.substr(line.find("=")+1));
                    }
                    else if (param.compare("p0y") == 0) {
                        P0(1) = std::stof(line.substr(line.find("=")+1));
                    }
                    else if (param.compare("p0z") == 0) {
                        P0(2) = std::stof(line.substr(line.find("=")+1));
                    }
                    else if (param.compare("p1x") == 0) {
                        P1(0) = std::stof(line.substr(line.find("=")+1));
                    }
                    else if (param.compare("p1y") == 0) {
                        P1(1) = std::stof(line.substr(line.find("=")+1));
                    }
                    else if (param.compare("p1z") == 0) {
                        P1(2) = std::stof(line.substr(line.find("=")+1));
                    }
                    else if (param.compare("angle") == 0) {
                        angle = std::stof(line.substr(line.find("=")+1));
                    }
                     
                    break;
                }
                case view: {
                    
                    if (param.compare("eye_posx") == 0) {
                        eye_pos(0) = std::stof(line.substr(line.find("=")+1));
                    }
                    else if (param.compare("eye_posy") == 0) {
                        eye_pos(1) = std::stof(line.substr(line.find("=")+1));
                    }
                    else if (param.compare("eye_posz") == 0) {
                        eye_pos(2) = std::stof(line.substr(line.find("=")+1));
                    }
                   
                    break;
                }
                case projection: {
                    
                    if (param.compare("eye_fov") == 0) {
                        eye_fov = std::stof(line.substr(line.find("=")+1));
                    }
                    else if (param.compare("aspect_ratio") == 0) {
                        aspect_ratio = std::stof(line.substr(line.find("=")+1));
                    }
                    else if (param.compare("zNear") == 0) {
                        zNear = std::stof(line.substr(line.find("=")+1));
                    }
                    else if (param.compare("zFar") == 0) {
                        zFar = std::stof(line.substr(line.find("=")+1));
                    }
                   
                    break;
                }
                case three_d_pts_and_inds: {
                    
                    if (param.compare("pt") == 0) {
                        pt = line.substr(line.find("=")+1);
                        comma_one_ind = pt.find(",");
                        comma_two_ind = pt.find(",", comma_one_ind+1);
                        pos.insert(pos.begin(), {std::stof(pt.substr(0,comma_one_ind)), std::stof(pt.substr(comma_one_ind+1, comma_two_ind-comma_one_ind-1)), std::stof(pt.substr(comma_two_ind+1))});
                    }
                    else if (param.compare("ind") == 0) {
                        pt = line.substr(line.find("=")+1);
                        comma_one_ind = pt.find(",");
                        comma_two_ind = pt.find(",", comma_one_ind+1);
                        ind.insert(ind.begin(), {std::stoi(pt.substr(0,comma_one_ind)), std::stoi(pt.substr(comma_one_ind+1, comma_two_ind-comma_one_ind-1)), std::stoi(pt.substr(comma_two_ind+1))});
                    }
                   
                    break;
                }
                default: break;
            }
        }
        // File Close
        ini_handler.close();
    }
    else {
        std::cout << "Unable to open the file!";
    }
    
}


Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // perspective + orthographic projection
    // Implement this function
    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    Eigen::Matrix4f pers_proj, orth_proj, projection;
    float n = zNear, f = zFar, t = abs(n)*tan((eye_fov/2) * M_PI / 180), b = -t, r = t*aspect_ratio, l = -r;
    // frustum -> cubic
    pers_proj << n, 0, 0, 0,
                 0, n, 0, 0,
                 0, 0, n+f, -f*n,
                 0, 0, 1, 0;

    // orthographic projection
    orth_proj << 2/(r-l), 0, 0, -(r+l)/(r-l),
                 0, 2/(t-b), 0, -(t+b)/(t-b),
                 0, 0, 2/(n-f), -(n+f)/(n-f),
                 0, 0, 0, 1;

    // squash all transformations
    projection = orth_proj * pers_proj;

    // std::clog << "projection" << std::endl << projection << std::endl; //check
    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "result.png";

    if (argc >= 2) {
        // ./main -r 0 output.png
        command_line = true;
        // angle = std::stof(argv[2]); 
        // -o by default
        if (argc == 3) {
            filename = std::string(argv[2]);
        }
    }

    rst::rasterizer r(1024, 1024);

    // define a triangle named by "pos" and "ind" (in 3D model coordinate) 
    std::vector<Eigen::Vector3f> pos;
    std::vector<Eigen::Vector3i> ind;
    // get_model_matrix
    Eigen::Vector3f T, S, P0, P1;
    // define your eye position "eye_pos" to a proper position (for view transformation)
    Eigen::Vector3f eye_pos; 
    // get_projection_matrix(float eye_fov, float aspect_ratio,float zNear, float zFar) (for projections)
    float eye_fov, aspect_ratio, zNear, zFar;

    parse_ini_params(T, S, P0, P1, eye_pos, eye_fov, aspect_ratio, zNear, zFar, pos, ind, angle);

    // ind-pos correspondance -> returning pos-ind-paired mapping ids
    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    // Eigen::Vector3f axis(0, 0, 1);
    Eigen::Vector3f axis(1, 0, 0);

    // gain input
    if (command_line) {
        // reinit frame and depth buffer
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle, T, S, P0, P1));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(eye_fov, aspect_ratio, zNear, zFar));

        // passing in id-pair to draw an object (segregated in triangles)
        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(1024, 1024, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }


    while (key != 27) {
        // draw a rotatable triangle in pixels
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle, T, S, P0, P1));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(eye_fov, aspect_ratio, zNear, zFar));

        // drawing triangle
        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(1024, 1024, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';
        std::clog << "angle: " << angle << std::endl;
        std::clog << "translation vector: \n" << T << std::endl;
        std::clog << "scaling vector: \n" << S << std::endl;
    

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }

        else if (key == 'j') {
            T(0) -= 1;
        }
        else if (key == 'l') {
            T(0) += 1;
        }
        else if (key == 'k') {
            T(1) -= 1;
        }
        else if (key == 'i') {
            T(1) += 1;
        }

        else if (key == 'f') {
            S(0) += 0.5;
        }
        else if (key == 'g') {
            S(1) += 0.5;
        }
        else if (key == 'h') {
            S(2) += 0.5;
        }
        else if (key == 'c') {
            S(0) -= 0.5;
        }
        else if (key == 'v') {
            S(1) -= 0.5;
        }
        else if (key == 'b') {
            S(2) -= 0.5;
        }
    }

    return 0;
}
