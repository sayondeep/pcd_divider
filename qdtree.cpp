#include<iostream>
#include<string>
#include"pointcloud_divider_node.hpp"

// Type alias for 2D matrix
using Matrix = std::vector<std::vector<int>>;


std::string pcd_name = "/home/sayon/autoware_map/town01/pointcloud_map.pcd";
std::string output_dir = "/home/sayon/autoware_map/town01/splitted/";
//std::string prefix = "t1";

template <class T>
bool checkEqual(const std::vector<std::vector<T>>& myList)
{
  if (myList.empty())
    return true;

  const std::vector<T>& first = myList[0];
  for (const std::vector<T>& x : myList)
  {
    if (x != first)
      return false;
  }

  return true;
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
    // static std::unordered_map<int, std::unordered_map<int, QuadTree>> tiles;
    // static std::unordered_map<int, int> count;

    int level;
    bool is_last;
    std::string parent_pcd;
    QuadTree* north_west;
    QuadTree* north_east;
    QuadTree* south_west;
    QuadTree* south_east;

public:
    QuadTree() : level(0), is_last(true), parent_pcd(""), north_west(nullptr), north_east(nullptr), south_west(nullptr), south_east(nullptr) 
    {
        // Initialize other member variables
    }

    void insert(std::string& pcd_name, std::vector<std::vector<int>> &matrix, int level = 0) 
    {
        //QuadTree quadTree;
        this->level = level;
        //quadTree.mean = calculate_mean(img).astype(int);
        //quadTree.resolution = {img.shape[0], img.shape[1]};
        //quadTree.img = img;
        this->parent_pcd = pcd_name;
        std::cout<<this->parent_pcd<<std::endl;
        // QuadTree::tiles[level][QuadTree::count[level]] = quadTree;
        // QuadTree::count[level] += 1;
        this->is_last = true;

        if (!checkEqual(matrix)) 
        {
            std::string prefix = std::to_string(level);
            
            std::vector<std::string> split_img = split4(pcd_name,output_dir,prefix);

            this->is_last = false;

            std::vector<std::vector<int>> n_w, n_e, s_w, s_e;
            splitMatrix(matrix, n_w, n_e, s_w, s_e);
            

            QuadTree* north_west = new QuadTree();
            north_west->insert(split_img[0], n_w, level + 1);

            QuadTree* north_east = new QuadTree();
            north_east->insert(split_img[1], n_e, level + 1);

            QuadTree* south_west = new QuadTree();
            south_west->insert(split_img[2], s_w, level + 1);

            QuadTree* south_east = new QuadTree();
            south_east->insert(split_img[3], s_e, level + 1);
        }

        return ;
    }

    void get_tiles(int level,std::vector<std::string> &tiles)
    {  
        std::cout<<"inside_get_tiles"<<std::endl;
        std::cout<<this->parent_pcd<<std::endl;
        
        if (this->is_last || this->level == level) 
        {
            //return np.tile(mean, (resolution[0], resolution[1], 1));
            // Display or process the image
            tiles.push_back(this->parent_pcd);
            std::cout<<this->parent_pcd<<std::endl;
        }

        if(this->north_west)
            this->north_west->get_tiles(level,tiles);
        if(this->north_west)
            this->north_east->get_tiles(level,tiles);
        if(this->south_west)
            this->south_west->get_tiles(level,tiles);
        if(this->south_east)
            this->south_east->get_tiles(level,tiles);

        // concatenate4(nw_image, ne_image, sw_image, se_image);

        return ;
    }
};


int main() 
{
    // Example usage
    //Image img;      // Provide the image

    // std::string pcd_name = "/home/sayon/autoware_map/town01/pointcloud_map.pcd";
    // std::string output_dir = "/home/sayon/autoware_map/town01/splitted/";
    // std::string prefix = "t1";
    
    std::vector<std::vector<int>> matrix ={{1,0,0,1},
                                            {0,0,0,0},
                                            {0,0,0,0},
                                            {0,0,0,0}};  // Provide the matrix

    QuadTree* qt = new QuadTree();
    qt->insert(pcd_name,matrix);
    std::vector<std::string> tiles;
    std::cout<<"calling_get_tiles"<<std::endl;
    qt->get_tiles(5,tiles);
    std::cout<<tiles.size()<<std::endl;
    for(std::string str:tiles)
    {
        std::cout<<str<<std::endl;
    }

    return 0;
}