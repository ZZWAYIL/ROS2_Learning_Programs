#include <iostream>
#include <thread> // 线程相关
#include <chrono> // 时间相关
#include <functional>
#include "cpp-httplib/httplib.h" // HTTP客户端库

class DownLoad
{
private:
    /* data */
public:
    void download(const std::string &host, const std::string &path,
                  const std::function<void(const std::string &, const std::string &)> &callback)
    {
        std::cout << "线程 " << std::this_thread::get_id() << " 开始下载 " << host + path << std::endl;
        httplib::Client cli(host.c_str());
        auto res = cli.Get(path.c_str());
        
        // 延迟2秒模拟下载时间
        // std::this_thread::sleep_for(std::chrono::seconds(2));

        if (res && res->status == 200) {
            callback(host + path, res->body);
        } else {
            std::cerr << "Failed to download " << host + path << std::endl;
        }
    }

    void download_start(const std::string &host, const std::string &path,
                        const std::function<void(const std::string &, const std::string &)> &callback)
    {
        std::thread download_thread(&DownLoad::download, this, host, path, callback);
        download_thread.detach(); // 分离线程，允许后台运行
    }
};

int main(){
    auto dl = DownLoad();
    auto word_count = [](const std::string &url, const std::string &content) ->void{
        std::cout << "Downloaded from " << url << ", content size: " << content.length() << " ->" << content.substr(0,9)<< std::endl;
    };

    dl.download_start("http://0.0.0.0:8000", "/novel1.txt", word_count);
    dl.download_start("http://0.0.0.0:8000", "/novel2.txt", word_count);
    dl.download_start("http://0.0.0.0:8000", "/novel3.txt", word_count);

    std::this_thread::sleep_for(std::chrono::seconds(10)); // 主线程等待，确保后台下载线程有时间完成
    return 0;
}
