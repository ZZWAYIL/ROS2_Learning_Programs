#include "iostream"
#include <memory>

int main(){
    auto num = 10;
    auto ptr = std::make_shared<std::string>("Hello Shared Pointer");
    std::cout << "共享指针引用次数：" << ptr.use_count() << std::endl;
    {
        auto ptr2 = ptr;
        std::cout << "共享指针引用次数：" << ptr.use_count() << std::endl;
    }
    std::cout << "共享指针引用次数：" << ptr.use_count() << std::endl;
    return 0;
}