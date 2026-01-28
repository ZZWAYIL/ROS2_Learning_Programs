#include "iostream"
#include "functional"

// 自由函数
void print_sum(int a, int b){
    std::cout << "The sum is: " << a + b << std::endl;
    std::cout << "This is from free function." << std::endl;
}

// 成员函数
class Calculator{
public:     
    void print_sum(int a, int b){
        std::cout << "The sum is: " << a + b << std::endl;
        std::cout << "This is from Calculator class." << std::endl;
    }
};



int main(){
    Calculator calc;
    // lambda表达式
    auto lambda_print_sum = [](int a, int b) {
        std::cout << "The sum is: " << a + b << std::endl;
        std::cout << "This is from lambda expression." << std::endl;
    };

    std::function<int(int,int)> add = [](int a,int b) -> int {
        return a + b;
    };

    print_sum(10, 20); // 调用自由函数
    calc.print_sum(10, 20); // 调用成员函数
    lambda_print_sum(10, 20); // 调用lambda表达式

    std::function<void(int,int)> save1 = print_sum;
    std::function<void(int,int)> save2 = lambda_print_sum;
    /**
     * @brief Creates a std::function object that binds the print_sum member function
     *        of a Calculator instance to a callable wrapper.
     * 
     * @details This binding uses std::bind to:
     *          - Bind the member function pointer &Calculator::print_sum
     *          - Bind the object instance &calc
     *          - Create placeholders for two integer parameters that will be 
     *            provided at call time
     * 
     * @usage Call save3 with two int arguments: save3(a, b)
     *        This will invoke calc.print_sum(a, b)
     * 
     * 翻译: 创建一个std::function对象，将Calculator实例的print_sum成员函数
     *      绑定到一个可调用的包装器。std::bind用于绑定成员函数指针、对象实例
     *      和两个整数参数的占位符。调用save3(a, b)时会执行calc.print_sum(a, b)
     */
    /**
     * @brief 使用std::bind将Calculator类的print_sum成员函数绑定到std::function对象
     * 
     * @details
     * - std::bind创建一个函数包装器，将print_sum方法与calc对象实例绑定
     * - std::placeholders::_1 表示绑定函数的第一个参数位置
     * - std::placeholders::_2 表示绑定函数的第二个参数位置
     * 
     * @note
     * 当调用save3(a, b)时，实际上会调用calc.print_sum(a, b)
     * _1和_2用于保留参数的原始位置，使得传入的参数能正确映射到目标函数
     */
    std::function<void(int,int)> save3 = std::bind(&Calculator::print_sum,
                &calc,std::placeholders::_1,std::placeholders::_2);

    save1(30, 40); // 通过std::function调用自由函数
    save2(30, 40); // 通过std::function调用lambda表达式
    save3(30, 40); // 通过std::function调用成员函数
    return 0;
}