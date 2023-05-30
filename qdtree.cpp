#include<iostream>
#include<string>
#include <fstream>
#include <climits>
#include"pointcloud_divider_node.hpp"

// Type alias for 2D matrix
using Matrix = std::vector<std::vector<int>>;


std::string pcd_name = "/home/sayon/autoware_map/town01/pointcloud_map.pcd";
std::string output_dir = "/home/sayon/autoware_map/town01/splitted/";
//std::string prefix = "t1";

template <class T>
bool checkEqual(const std::vector<std::vector<T>>& matrix)
{
    if (matrix.empty()) {
        return true;
    }

    int firstElement = matrix[0][0];

    for (const auto& row : matrix) {
        for (int element : row) {
            if (element != firstElement) {
                return false;
            }
        }
    }

    return true;
}


std::vector<std::vector<int>> readMatrixFromFile(const std::string& filename) {
    std::ifstream inputFile(filename);
    std::vector<std::vector<int>> matrix;

    if (inputFile) {
        int rows, cols;
        inputFile >> rows >> cols;  // Read the dimensions of the matrix

        // Resize the matrix
        matrix.resize(rows, std::vector<int>(cols));

        // Read the matrix elements
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                inputFile >> matrix[i][j];
            }
        }

        inputFile.close();
    } else {
        std::cout << "Failed to open the file: " << filename << std::endl;
    }

    return matrix;
}

// Function to split a matrix into four quadrants
void splitMatrix(const Matrix& inputMatrix, Matrix& quadrant1, Matrix& quadrant2, Matrix& quadrant3, Matrix& quadrant4) 
{
    int rows = inputMatrix.size();
    int cols = inputMatrix[0].size();

    // Calculate the half size of the matrix
    int halfRows = rows / 2;
    int halfCols = cols / 2;

    // Resize the quadrants
    quadrant1.resize(halfRows, std::vector<int>(halfCols));
    quadrant2.resize(halfRows, std::vector<int>(halfCols));
    quadrant3.resize(halfRows, std::vector<int>(halfCols));
    quadrant4.resize(halfRows, std::vector<int>(halfCols));

    // Split the matrix into quadrants
    for (int i = 0; i < halfRows; ++i) {
        for (int j = 0; j < halfCols; ++j) {
            quadrant1[i][j] = inputMatrix[i][j];                                      // Top-left quadrant
            quadrant2[i][j] = inputMatrix[i][j + halfCols];                           // Top-right quadrant
            quadrant3[i][j] = inputMatrix[i + halfRows][j];                           // Bottom-left quadrant
            quadrant4[i][j] = inputMatrix[i + halfRows][j + halfCols];                // Bottom-right quadrant
        }
    }
}

// // Function to display a matrix
// void displayMatrix(const Matrix& matrix) {
//     for (const auto& row : matrix) {
//         for (const auto& element : row) {
//             std::cout << element << " ";
//         }
//         std::cout << std::endl;
//     }
// }

class QuadTree 
{
private:

    int level;
    bool is_last;
    std::string parent_pcd;
    std::pair<double,double> low , high;
    QuadTree* north_west;
    QuadTree* north_east;
    QuadTree* south_west;
    QuadTree* south_east;

public:
    QuadTree() : level(0), low(INT_MIN,INT_MIN),high(INT_MAX,INT_MAX),is_last(true), parent_pcd(""), north_west(nullptr), north_east(nullptr), south_west(nullptr), south_east(nullptr) 
    {
        // Initialize other member variables
    }

    void insert(std::string& pcd_name, std::vector<std::vector<int>> &matrix, int level = 0) 
    {
        this->level = level;

        this->parent_pcd = pcd_name;

        std::vector<double> bound = calculateGridSize(this->parent_pcd);

        this->low.first = bound[2];
        this->low.second = bound[3];

        this->high.first = bound[4];
        this->high.second = bound[5];

        this->is_last = true;

        if (!checkEqual(matrix)) 
        {
            std::string prefix = std::to_string(level);
            
            std::vector<std::string> split_img = split4(pcd_name,output_dir,prefix,bound);

            this->is_last = false;

            std::vector<std::vector<int>> n_w, n_e, s_w, s_e;
            splitMatrix(matrix, n_w, n_e, s_w, s_e);
            

            //QuadTree* north_west = new QuadTree();
            this->north_west = new QuadTree();
            this->north_west->insert(split_img[0], n_w, level + 1);

            this-> north_east = new QuadTree();
            this->north_east->insert(split_img[1], n_e, level + 1);

            this-> south_west = new QuadTree();
            this->south_west->insert(split_img[2], s_w, level + 1);

            this-> south_east = new QuadTree();
            this->south_east->insert(split_img[3], s_e, level + 1);
        }
        // else
        // {
        //     for (const auto& row : matrix) 
        //     {
        //         for (const auto& element : row) 
        //         {
        //             std::cout << element << " ";
        //         }
        //         std::cout << std::endl;
        //     }
        // }

        return ;
    }

    void get_tiles(int level,std::vector<std::string> &tiles)
    {  
        
        if (this->is_last || this->level == level) 
        {
            // Display or process the image
            tiles.push_back(this->parent_pcd);
            //std::cout<<this->parent_pcd<<std::endl;
        }

        if(this->north_west)
            this->north_west->get_tiles(level,tiles);
        if(this->north_east)
            this->north_east->get_tiles(level,tiles);
        if(this->south_west)
            this->south_west->get_tiles(level,tiles);
        if(this->south_east)
            this->south_east->get_tiles(level,tiles);

        return ;
    }

    std::string get_tile_by_loc(QuadTree* qt, double x, double y)
    {
        if (qt->is_last) 
        {
            return qt->parent_pcd;

        }
        
        if (x < (qt->low.first + qt->high.first) / 2) 
        {
            if (y < (qt->low.second + qt->high.second) / 2) 
            {
                return get_tile_by_loc(qt->south_west, x,  y);
            } 
            else 
            {
                return get_tile_by_loc(qt->north_west, x,  y);
            }
        } 
        else 
        {
            if (y < (qt->low.second + qt->high.second) / 2) 
            {
                return get_tile_by_loc(qt->south_east, x,  y);
            } 
            else 
            {
                return get_tile_by_loc(qt->north_east, x,  y);
            }
        }

    }
};


void give_tile(QuadTree* qt,double x,double y)
{
    std::cout<<"The corres. tile name is "<<qt->get_tile_by_loc(qt,x,y)<<std::endl;
}


int main() 
{
    // std::string pcd_name = "/home/sayon/autoware_map/town01/pointcloud_map.pcd";
    // std::string output_dir = "/home/sayon/autoware_map/town01/splitted/";
    // std::string prefix = "t1";
    
    std::vector<std::vector<int>> matrix = readMatrixFromFile("../matrix.txt");
    // Display the matrix
    // for (const auto& row : matrix) 
    // {
    //     for (const auto& element : row) 
    //     {
    //         std::cout << element << " ";
    //     }
    //     std::cout << std::endl;
    // }
    // {{1,0,0,1},
    //                                         {0,0,0,0},
    //                                         {0,0,0,0},
    //                                         {0,0,0,0}};  // Provide the matrix

    QuadTree* qt = new QuadTree();
    qt->insert(pcd_name,matrix);
    
    std::vector<std::string> tiles;

    qt->get_tiles(7,tiles);
    std::cout<<tiles.size()<<std::endl;
    // for(std::string str:tiles)
    // {
    //     std::cout<<str<<std::endl;
    // }
    std::ofstream outputFile("../pcd_tiles.txt");  // Open the file for writing

    if (outputFile.is_open()) 
    {
        // Iterate over the vector and write each string to the file
        for (const std::string& str : tiles) 
        {
            std::cout<<str<<std::endl;
            outputFile << str << std::endl;
        }

        outputFile.close();  // Close the file
        std::cout << "Tiles saved to file." << std::endl;
    } 
    else 
    {
        std::cout << "Error opening the file." << std::endl;
    }


    while(true)
    {
        int ch;
        std::cout<<"Enter 1 to give location or 0 to exit"<<std::endl;
        std::cin>>ch;

        if(ch==0)
            break;
        else
        {
            double x,y;
            std::cout<<"Enter X:"<<std::endl;
            std::cin>>x;
            std::cout<<"Enter Y:"<<std::endl;
            std::cin>>y;

            give_tile(qt,x,y);
        }
             
    }

    return 0;
}
