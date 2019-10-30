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
 # 也支持markdown
 ## 参数说明如下表：
 name     | type     |description of param
 ----------|-----------|--------------------
 car_id   | int      |车源编号
 province | int      |业务员所在省份
 x        |  x       |   x
 x        |  x       |   x
 x        |  x       |   x
 @return    返回值说明如下：
 name     | type     | description of value
 -------- |----------|----------------------
 car_id   | int      | 车源编号
 car_info | object   | json对象格式的车源信息

 > 所以也可以用markdown,'*'也不用每行都写


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
	 *  @retval 1 xxx
	 *  @retval 2 xxx
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
