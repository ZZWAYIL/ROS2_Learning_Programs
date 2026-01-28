import threading
import requests

class DownLoad:
    def download(self,url,callback_word_count):
        print(f"线程{threading.get_ident()}开始下载{url}内容")
        response = requests.get(url)
        response.encoding = 'utf-8'
        result = response.text
        callback_word_count(url,result)

    def download_start(self,url,callback_word_count):
        thread = threading.Thread(target = self.download,args=(url,callback_word_count))
        thread.start()


def word_count(url,result):
    print(f"{url}的字数是{len(result)}")


def main():
    download = DownLoad()
    download.download_start("http://0.0.0.0:8000/novel1.txt",word_count)
    download.download_start("http://0.0.0.0:8000/novel2.txt",word_count)
    download.download_start("http://0.0.0.0:8000/novel3.txt",word_count)