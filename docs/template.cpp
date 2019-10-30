/** @file template.cpp
 * @version 0.0.0
 * @author zgh
 * @todo 写其他文档，参见task.md
 * @brief 文件简述，file标识文件名
 * 详细说明直接跟在下面
 * ...
 * ...
 * @section 第一章：相当于word的标题
 * ...
 * @section 第二章：相当于word的标题
 * ...
 * @date 20191030
 */

#include <bits/stdc++.h>
using namespace std;



/**
* @class TestDoxygen template.cpp
* @brief 类简述
* 详细说明
* @section 第一节：相当于word的标题
* ...
* @section 第二节：相当于word的标题
* ...
* ...
* ...
* @author xxx
* @note 注意
* detailed description
*/
class TestDoxygen{
private:
	int m_id;///< 这个是数据的注释
public:
	/**
	 * @brief 这个是测试函数注释
 	 * @section 第一部分：相当于word的标题
	 * ...
	 * @note 注意。。。
	 * @param[in] 输入str
	 * 	-例1 p1
	 * 	-例2 p2
	 * @param[out] 输出 void没有输出
	 * @return 和param写法类似
	 * @see 注意
	 */
	static void printHello(string str){
		cout << "hello" << str << endl; 
	}
}

int main(){
	TestDoxygen::printHello(" ");
	return 0;
}
