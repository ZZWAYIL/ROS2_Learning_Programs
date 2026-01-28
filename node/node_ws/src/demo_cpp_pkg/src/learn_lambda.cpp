#include "iostream"
#include <algorithm>

int main(){
    auto add = [](int a,int b) -> int {
        return a + b;
    };

    int res = add(10,20);
    auto print_sum = [res]()-> void{
        std::cout << "The sum is: " << res << std::endl;
    };
    print_sum();

    return 0;
}