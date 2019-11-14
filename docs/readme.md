# Doxygen 生成
ubuntu下：
> sudo apt-get install doxygen

## 生成html
将需要生成文档文件写入FileList中，然后执行
> python GenDoc.py

生成文档即在html中，pdf版本获取需要打开配置文件中的latex选项，根据latex生成

## 生成pdf
### 安装latex
> sudo apt-get install texlive-full 
> sudo apt-get install cjk-latex

### 生成pdf
在执行 ``` python GenDoc.py ``` 之后,等待指令执行，执行完毕在latex文件夹下便有生成的refman.pdf

# C++ Doxygen格式
template.cpp中有一个简单的示例说明，可以打开/html/index.html查看对应关系，目前只需要完成三点：
- 文件说明
- 类说明
- 函数说明
