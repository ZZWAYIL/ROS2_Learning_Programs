#include "iostream"
// #include "rclcpp.h"

int main(int argc, char * argv[]){
    std::cout << "参数数量 = "<< argc << std::endl;
    std::cout << "程序名称 = " << argv[0] << std::endl;

    std::string str1;
    str1 = argv[1]; 
    if(str1 == "--help"){
        std::cout << "这是一个帮助信息" << std::endl;
    }

    return 0;
}