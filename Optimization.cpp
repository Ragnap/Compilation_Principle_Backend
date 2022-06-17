/**
 * @Author       : RagnaLP
 * @Date         : 2022-06-13 11:38:40
 * @LastEditTime : 2022-06-18 00:14:18
 * @Description  : 后端程序
 */
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>
using namespace std;
#define DEBUG
//最大标记数量
const int MARK_SIZE = 1000;
//最大附加标记数量
const int ADD_MARK_SIZE = 100;
//最大四元式数量
const int QUAT_SIZE = 1000;
//最大分块数量
const int BLOCK_SIZE = 1000;
//寄存器数量
const int REG_SIZE = 4;
//目标代码生成路径
const char* const TARGET_PATH = "target.txt";
// DAG调试数据生成路径
const char* const DEBUG_DAG_PATH = "debug/DAG.txt";
// 活跃信息调试数据生成路径
const char* const DEBUG_ACTIVE_PATH = "debug/active.txt";
// 分块调试数据生成路径
const char* const DEBUG_BLOCK_PATH = "debug/block.txt";
// 目标语言调试数据生成路径
const char* const DEBUG_TARGET_PATH = "debug/target.txt";

//////////////////////////////////四元式类//////////////////////////////
class Quaternary {
public:
    void clear() {
        ope.clear();
        mark_1.clear();
        mark_2.clear();
        result.clear();
    }
    string ope;
    string mark_1;
    string mark_2;
    string result;
};
//得到一个四元式,提供的是字符串
Quaternary buildQuaternary(string ope, string mark_1, string mark_2, string result) {
    Quaternary res;
    res.ope = ope;
    res.mark_1 = mark_1;
    res.mark_2 = mark_2;
    res.result = result;
    return res;
}
// 输出一个四元式
void printQuaternary(ostream& out, Quaternary quat) {
    out << '(' << quat.ope << "    \t";
    out << "   \t";
    out << quat.mark_2;
    out << "   \t";
    out << quat.result;
    out << "   \t";
    out << ')' << endl;
}
// 四元式记数
int quatCnt = 0;
//////////////////////////////////类型部分//////////////////////////////

//常数类型枚举变量，以字节数逐渐增大
enum ConstKind { NOT_CONST, BOOL, CHAR, INT, DOUBLE };
//从符号表确定类型
class KindMap {
public:
    KindMap() {
        ifstream synbl("synbl.txt");
        string input, mark, kind;
        getline(synbl, input);
        getline(synbl, input);
        getline(synbl, input);
        getline(synbl, input);
        while(input.size()) {
            mark.clear();
            kind.clear();
            int pos = 0;
            while(input[pos] != '\t' && input[pos] != ' ') {
                mark += input[pos++];
            }
            while(input[pos] == '\t' || input[pos] == ' ') {
                pos++;
            }
            while(input[pos] != '\t' && input[pos] != ' ') {
                kind += input[pos++];
            }
            if(kind == "bool")
                markMap[mark] = BOOL;
            if(kind == "char")
                markMap[mark] = CHAR;
            if(kind == "int")
                markMap[mark] = INT;
            if(kind == "double")
                markMap[mark] = DOUBLE;
            if(kind == "float")
                markMap[mark] = DOUBLE;
            getline(synbl, input);
        }
        synbl.close();
    }
    bool inMap(string markString) {
        return (markMap.find(markString) != markMap.end());
    }
    ConstKind getKind(string markString) {
        return markMap[markString];
    }

private:
    map<string, ConstKind> markMap;
} kindMap;

//判断一个字符串的常数类型
ConstKind checkConstKind(string str) {
    if(str == "_")
        return NOT_CONST;
    if(isalpha(str[0]))
        return NOT_CONST;
    else if(str[0] == '\'')
        return CHAR;
    else {
        for(int i = 0; i < str.length(); i++)
            if(str[i] == '.')
                return DOUBLE;
        return INT;
    }
}

//////////////////////////////////常量计算部分///////////////////////////
//获取常量计算式计算结果
// bool 单目 这里只有 !
bool calcConstValue(bool a) {
    return !a;
}
// 其他单目 这里只有 -
template<typename T> T calcConstValue(T a) {
    return -a;
}
// bool 双目
bool calcConstValue(bool a, bool b, string ope) {
    if(ope == "&&")
        return a & b;
    if(ope == "||")
        return a | b;
    return 0;
}
// char 双目
char calcConstValue(char a, char b, char ope) {
    switch(ope) {
        case '+':
            return a + b;
            break;
        case '-':
            return a - b;
            break;
    }
    return 0;
}
// int 双目
int calcConstValue(int a, int b, char ope) {
    switch(ope) {
        case '+':
            return a + b;
            break;
        case '-':
            return a - b;
            break;
        case '*':
            return a * b;
            break;
        case '/':
            return a / b;
            break;
        case '%':
            return a % b;
            break;
        case '&':
            return a & b;
            break;
        case '|':
            return a | b;
            break;
        case '^':
            return a ^ b;
            break;
    }
    return 0;
}
// double 双目
double calcConstValue(double a, double b, char ope) {
    switch(ope) {
        case '+':
            return a + b;
            break;
        case '-':
            return a - b;
            break;
        case '*':
            return a * b;
            break;
        case '/':
            return a / b;
            break;
    }
    return 0;
}
// 比较 双目
template<typename T> bool calcConstCompareValue(T a, T b, string ope) {
    if(ope == "<")
        return a < b;
    if(ope == "<=")
        return a <= b;
    if(ope == ">")
        return a > b;
    if(ope == ">=")
        return a >= b;
    if(ope == "==")
        return a == b;
    if(ope == "!=")
        return a != b;
    return 0;
}
//////////////////////////////////标记部分//////////////////////////////

//标记类
class Mark {
public:
    //创建一个新的标记
    void init(string label) {
        markString = label;
        //判断是否为临时标记
        tempMark = (label[0] == 't');
        //判断一个标记的常数类型
        constKind = checkConstKind(markString);
        lastNodeID = 0;
    }
    //设置在图上的节点编号
    void setNodeID(int node_ID) {
        lastNodeID = node_ID;
    }
    //获取在图上节点编号
    int getNodeID() {
        return lastNodeID;
    }
    //获取该标记的常数类型
    ConstKind getConstKind() {
        return constKind;
    }
    //获取该标记的常数表下标
    int getConstKindIndex() {
        return constKindIndex;
    }
    //获取是否为临时标记
    bool isTempMark() {
        return tempMark;
    }  //原字符串
    string markString;

private:
    //常量类型
    ConstKind constKind;
    //常值表下标
    int constKindIndex;
    //在图上的节点编号
    int lastNodeID;
    //标记是否为临时变量
    bool tempMark;
} mark[MARK_SIZE];
//标记字符串与标记编号的映射
map<string, int> markID;
//标记数量
int markCnt = 0;
//获取一个标记的编号,对于第一次出现的标记自动初始化,若新建标记，isNew=1
int getMarkID(string markString, bool& isNew) {
    if(markID.find(markString) != markID.end()) {  //已有编号
        isNew = 0;
        return markID[markString];
    }
    else {
        isNew = 1;
        //更新标记表
        mark[++markCnt].init(markString);
        //分配新的标记编号
        return markID[markString] = markCnt;
    }
}
//获取一个标记的编号,对于第一次出现的标记自动初始化
int getMarkID(string markString) {
    if(markID.find(markString) != markID.end()) {  //已有编号
        return markID[markString];
    }
    else {
        //更新标记表
        mark[markCnt].init(markString);
        //分配新的标记编号
        return markID[markString] = markCnt++;
    }
}
//判断一个标号的类型
ConstKind checkMarkKind(string str) {
    if(str == "_")
        return NOT_CONST;
    if(markID.find(str) == markID.end())
        return NOT_CONST;
    else
        return checkConstKind(str);
}
//////////////////////////////////节点部分//////////////////////////////

// 结点类
class Node {
public:
    //重置
    void clear() {
        mainMark = leftNodeID = rightNodeID = 0;
        tempMainMark = 0;
        additionMark.clear();
        opera.clear();
    }
    //设置主标记
    void setMainMark(int mark_ID, bool is_temp) {
        mainMark = mark_ID;
        tempMainMark = is_temp;
    }
    //获取主标记
    int getMainMark() {
        return mainMark;
    }
    //查找某个标记编号是否在此节点中
    bool inNode(int mark_ID) {
        if(mark_ID == mainMark)
            return true;
        for(int i = 0; i < additionMark.size(); i++) {
            if(mark_ID == additionMark[i])
                return true;
        }
        return false;
    }
    //在附加标记中删除某个标记编号
    void delAdditionMark(int mark_ID) {
        for(int i = 0; i < additionMark.size(); i++) {
            if(mark_ID == additionMark[i]) {
                additionMark.erase(additionMark.begin() + i);
                return;
            }
        }
    }
    //加入附加标记
    void addAdditionMark(int mark_ID, bool is_temp) {
        //如果已经在附加标记中就不再加入
        if(inNode(mark_ID))
            return;
        //当主标记是临时标记但新加入的附加标记不是临时标记时进行交换
        if(tempMainMark && !is_temp) {
            additionMark.push_back(mainMark);
            mainMark = mark_ID;
            tempMainMark = 0;
        }
        else
            additionMark.push_back(mark_ID);
    }
    //左操作数节点编号
    int leftNodeID;
    //右操作数节点编号
    int rightNodeID;
    //操作符
    string opera;
    //附加标记编号表
    vector<int> additionMark;

private:
    //主标记编号
    int mainMark;
    //主标记是否为临时标记
    bool tempMainMark;
};

//////////////////////////////////DAG部分//////////////////////////////

// DAG类
class DAG_optimization {
public:
    DAG_optimization() {
#ifdef DEBUG
        debug.open(DEBUG_DAG_PATH);
#endif
    }
    //重构四元式
    vector<Quaternary> generate() {
        vector<Quaternary> result;
        //先标记需要用的表达式
        for(int i = nodeCnt; i; i--) {
            bool need = needNode[i];
            need |= (!mark[node[i].getMainMark()].isTempMark());
            for(int j = 0; j < node[i].additionMark.size(); j++) {
                if(!mark[node[i].additionMark[j]].isTempMark()) {
                    need = 1;
                    break;
                }
            }
            if(need) {
                needNode[i] = 1;
                if(node[i].leftNodeID != 0)
                    needNode[node[i].leftNodeID] = 1;
                if(node[i].rightNodeID != 0)
                    needNode[node[i].rightNodeID] = 1;
            }
        }
        //再进行输出
        for(int i = 1; i <= nodeCnt; i++) {
            if(needNode[i]) {
                if(node[i].leftNodeID || node[i].rightNodeID) {
                    result.push_back(buildQuaternary(node[i].opera, mark[node[node[i].leftNodeID].getMainMark()].markString, mark[node[node[i].rightNodeID].getMainMark()].markString, mark[node[i].getMainMark()].markString));
                }
                for(int j = 0; j < node[i].additionMark.size(); j++) {
                    if(!mark[node[i].additionMark[j]].isTempMark()) {
                        result.push_back(buildQuaternary("=", mark[node[i].getMainMark()].markString, "_", mark[node[i].additionMark[j]].markString));
                    }
                }
            }
        }
        // 特判输出块结尾
        if(blockEnd.ope.size()) {
            result.push_back(blockEnd);
        }
        return result;
    }
    //从输入的四元式构建DAG的一条边
    void addEdge(Quaternary inputQuaternary) {
        string opera = inputQuaternary.ope, mark_1 = inputQuaternary.mark_1, mark_2 = inputQuaternary.mark_2, result = inputQuaternary.result;

        if(mark_1 == "_" && mark_2 != "_")
            swap(mark_1, mark_2);
        int markID_1, markID_2, markID_3;
        int nodeID_1, nodeID_2, nodeID_3;
        bool newMark = 0;
        string topMark_1 = getNodeMainMarkString(mark_1);
        string topMark_2 = getNodeMainMarkString(mark_2);
        ConstKind kind_1 = checkConstKind(topMark_1);
        ConstKind kind_2 = checkConstKind(topMark_2);
        //块结束符
        if(result == "_") {
            if(mark_1 != "_") {
                //此时的mark_1会被用来判断块的转向,将其主标记need标志设为1
                markID_1 = getMarkID(mark_1, newMark);
                if(newMark) {  //新建节点
                    buildNode(markID_1);
                }
                needNode[mark[markID_1].getNodeID()] = 1;
            }
            blockEnd = inputQuaternary;
            return;
        }
        //单目运算常值表达式
        else if(opera != "=" && kind_1 != NOT_CONST && mark_2 == "_") {
            string temp;
            kind_1 = kindMap.getKind(mark_1);
            switch(kind_1) {
                case BOOL:
                    temp = to_string(calcConstValue((bool)stoi(topMark_1)));
                    break;
                case CHAR:
                    temp = to_string(calcConstValue(stoi(topMark_1)));
                    break;
                case INT:
                    temp = to_string(calcConstValue(stoi(topMark_1)));
                    break;
                case DOUBLE:
                    temp = to_string(calcConstValue(stof(topMark_1)));
                    break;
            }
            markID_1 = getMarkID(temp, newMark);
            if(newMark) {  //新建节点
                buildNode(markID_1);
            }
            markID_3 = getMarkID(result, newMark);
            for(int i = 1; i <= nodeCnt; i++) {
                node[i].delAdditionMark(markID_3);
            }
            nodeID_1 = mark[markID_1].getNodeID();
            node[nodeID_1].addAdditionMark(markID_3, mark[markID_3].isTempMark());
            mark[markID_3].setNodeID(nodeID_1);
        }
        //双目运算常值表达式
        else if(opera != "=" && kind_1 != NOT_CONST && kind_2 != NOT_CONST) {
            string temp;
            kind_1 = kindMap.getKind(mark_1);
            kind_2 = kindMap.getKind(mark_2);
            //双目比较运算
            if(opera == ">=" || opera == ">" || opera == "<=" || opera == "<" || opera == "==" || opera == "!=") {
                ConstKind resKind = max(kind_1, kind_2);
                switch(resKind) {
                    case BOOL:
                        temp = to_string(calcConstCompareValue(stoi(topMark_1), stoi(topMark_2), opera));
                        break;
                    case CHAR:
                        temp = to_string(calcConstCompareValue(stoi(topMark_1), stoi(topMark_2), opera));
                        break;
                    case INT:
                        temp = to_string(calcConstCompareValue(stoi(topMark_1), stoi(topMark_2), opera));
                        break;
                    case DOUBLE:
                        temp = to_string(calcConstCompareValue(stof(topMark_1), stof(topMark_2), opera));
                        break;
                }
            }
            //双目算术运算
            else {
                ConstKind resKind = max(kind_1, kind_2);
                switch(resKind) {
                    case BOOL:
                        ///////////
                    case CHAR:
                        temp = to_string(calcConstValue(stoi(topMark_1), stoi(topMark_2), opera[0]));
                        break;
                    case INT:
                        temp = to_string(calcConstValue(stoi(topMark_1), stoi(topMark_2), opera[0]));
                        break;
                    case DOUBLE:
                        temp = to_string(calcConstValue(stof(topMark_1), stof(topMark_2), opera[0]));
                        break;
                }
            }
            markID_1 = getMarkID(temp, newMark);
            if(newMark) {  //新建节点
                buildNode(markID_1);
            }
            markID_3 = getMarkID(result, newMark);
            for(int i = 1; i <= nodeCnt; i++) {
                node[i].delAdditionMark(markID_3);
            }
            nodeID_1 = mark[markID_1].getNodeID();
            node[nodeID_1].addAdditionMark(markID_3, mark[markID_3].isTempMark());
            mark[markID_3].setNodeID(nodeID_1);
        }

        //赋值运算
        else if(opera == "=") {
            markID_1 = getMarkID(mark_1, newMark);
            if(newMark) {  //新建节点
                buildNode(markID_1);
            }
            if(mark_1[0] == 't' || kind_1 == NOT_CONST) {
                markID_3 = getMarkID(result, newMark);
                for(int i = 1; i <= nodeCnt; i++) {
                    node[i].delAdditionMark(markID_3);
                }
                nodeID_1 = mark[markID_1].getNodeID();
                node[nodeID_1].addAdditionMark(markID_3, mark[markID_3].isTempMark());
                mark[markID_3].setNodeID(nodeID_1);
            }
            else {  //对主标记中常量赋值应该删去之前的主标记
                markID_3 = getMarkID(result, newMark);
                for(int i = 1; i <= nodeCnt; i++) {
                    if(node[i].getMainMark() == markID_3) {  //从附加标记中选取一个标记作为主标记
                        //优先选取非临时变量
                        int pos;
                        for(pos = 0; pos < node[i].additionMark.size(); pos++) {
                            if(!mark[node[i].additionMark[pos]].isTempMark()) {
                                break;
                            }
                        }
                        if(pos == node[i].additionMark.size()) {  //附加标记中没有非临时变量
                            node[i].setMainMark(node[i].additionMark.back(), mark[node[i].additionMark.back()].isTempMark());
                            node[i].additionMark.pop_back();
                        }
                        else {
                            node[i].setMainMark(node[i].additionMark[pos], mark[node[i].additionMark[pos]].isTempMark());
                            node[i].additionMark.erase(node[i].additionMark.begin() + pos);
                        }
                    }
                    else
                        node[i].delAdditionMark(markID_3);
                }
                nodeID_1 = mark[markID_1].getNodeID();
                node[nodeID_1].addAdditionMark(markID_3, mark[markID_3].isTempMark());
                mark[markID_3].setNodeID(nodeID_1);
            }
        }

        else {
            markID_1 = getMarkID(mark_1, newMark);
            if(newMark)
                buildNode(markID_1);

            markID_2 = getMarkID(mark_2, newMark);
            if(newMark)
                buildNode(markID_2);

            markID_3 = getMarkID(result, newMark);
            int sameExpression = 0;
            //检查是否有公共表达式，并删除重复定义的标记
            for(int i = nodeCnt; i; i--) {
                node[i].delAdditionMark(markID_3);
                if(node[i].opera != opera)
                    continue;
                if(!sameExpression && node[i].leftNodeID == mark[markID_1].getNodeID() && node[i].rightNodeID == mark[markID_2].getNodeID()) {
                    sameExpression = i;
                }
            }
            if(sameExpression) {  //存在公共表达式
                node[sameExpression].addAdditionMark(markID_3, mark[markID_3].isTempMark());
                mark[markID_3].setNodeID(sameExpression);
            }
            else {
                nodeID_1 = mark[markID_1].getNodeID();
                nodeID_2 = mark[markID_2].getNodeID();
                buildNode(markID_3, nodeID_1, nodeID_2, opera);
            }
        }
#ifdef DEBUG
        check();
#endif
    }
    //重置DAG
    void clear() {
        markID.clear();
        markCnt = 0;
        for(int i = 0; i <= nodeCnt; i++) {
            node[i].clear();
            needNode[i] = 0;
        }
        nodeCnt = 0;
        blockEnd.clear();
    }

private:
    //特殊的块结束符
    Quaternary blockEnd;
    //创建新节点
    void buildNode(int mainMark_ID, int leftNodeID = 0, int rightNodeID = 0, string ope = "") {
        //创建图上的新节点
        node[++nodeCnt].setMainMark(mainMark_ID, mark[mainMark_ID].isTempMark());
        node[nodeCnt].opera = ope;
        node[nodeCnt].leftNodeID = leftNodeID;
        node[nodeCnt].rightNodeID = rightNodeID;

        mark[mainMark_ID].setNodeID(nodeCnt);
    }
    //从操作符映射到节点的主要编号，对主要标记进行运算
    string getNodeMainMarkString(string markString) {
        // return markString;
        if(markString == "_")
            return markString;
        if(markID.find(markString) == markID.end())
            return markString;
        if(mark[markID[markString]].getNodeID() == 0)
            return markString;
        return mark[node[mark[markID[markString]].getNodeID()].getMainMark()].markString;
    }
    //节点数组
    Node node[MARK_SIZE];
    //节点数量
    int nodeCnt = 0;
    //节点有效标记
    bool needNode[MARK_SIZE];
#ifdef DEBUG
    // 调试用文件流
    ofstream debug;
    // 调试用进度追踪
    int line;
    // DAG检查调试用函数
    void check() {
        debug << "####### now finish line:" << ++line << endl;
        for(int i = nodeCnt; i; i--) {
            debug << "Node id:" << i << endl;
            debug << "l: " << node[i].leftNodeID << "  r:" << node[i].rightNodeID << endl;
            debug << "operator: " << node[i].opera << endl;
            debug << "have: " << mark[node[i].getMainMark()].markString << "|";
            for(int j = 0; j < node[i].additionMark.size(); j++) {
                debug << " " << mark[node[i].additionMark[j]].markString;
            }
            debug << endl << endl;
        }
        debug << " *** mark postion:" << endl;
        for(int i = 1; i < markCnt; i++) {
            debug << "\t" << mark[i].markString << "\t at \t" << mark[i].getNodeID() << endl;
        }
        debug << "###################" << endl;
        debug << endl;
    }
#endif
} DAG;

////////////////////////////////目标代码部分////////////////////////////

//构建时的活跃信息描述表
bool needActive[MARK_SIZE];
//活跃信息记录
bool isActive[QUAT_SIZE][3];
//从四元式到文法的翻译类
class Translater {
public:
    Translater() {
#ifdef DEBUG
        debug.open(DEBUG_TARGET_PATH);
#endif
    }
    //加入一个块的四元式
    void addBlock(vector<Quaternary> blockQua) {
        string ope, mark_1, mark_2, result, temp;
        int markID_1, markID_2, markID_3;
        clearRegister();
        for(int line = 0; line < blockQua.size(); line++) {
            ope = blockQua[line].ope;
            mark_1 = blockQua[line].mark_1;
            mark_2 = blockQua[line].mark_2;
            result = blockQua[line].result;
            //单目运算符
            if(mark_2 == "_" && (ope == "-" || ope == "!")) {
                //更新活跃信息
                nowActive[getMarkID(mark_1)] = isActive[line][0];
                nowActive[getMarkID(result)] = isActive[line][2];

                allocateRegister(blockQua[line], 0);

                //保存式子
                if(ope == "-")
                    temp = "NEG\tR" + to_string(useRegister[0]);
                else if(ope == "!")
                    temp = "NO\tR" + to_string(useRegister[0]);
                target.push_back(temp);
            }
            //双目不可交换运算符
            else if(ope == "-" || ope == "/" || ope == ">=" || ope == ">" || ope == "<=" || ope == "<") {
                //更新活跃信息
                nowActive[getMarkID(mark_1)] = isActive[line][0];
                nowActive[getMarkID(mark_2)] = isActive[line][1];
                nowActive[getMarkID(result)] = isActive[line][2];

                allocateRegister(blockQua[line], 0);

                //如果mark_2在寄存器里，就使用寄存器
                if(useRegister[1] != -1) {
                    mark_2 = "R" + to_string(useRegister[1]);
                }
                //保存式子
                if(ope == "-")
                    temp = "SUB\tR" + to_string(useRegister[0]) + "\t," + mark_2;
                else if(ope == "/")
                    temp = "DIV\tR" + to_string(useRegister[0]) + "\t," + mark_2;
                else if(ope == ">=")
                    temp = "GE\tR," + to_string(useRegister[0]) + "\t," + mark_2;
                else if(ope == ">")
                    temp = "GT\tR," + to_string(useRegister[0]) + "\t," + mark_2;
                else if(ope == "<=")
                    temp = "LE\tR," + to_string(useRegister[0]) + "\t," + mark_2;
                else if(ope == "<")
                    temp = "LT\tR," + to_string(useRegister[0]) + "\t," + mark_2;
                target.push_back(temp);
            }
            //双目可交换运算符
            else if(ope == "+" || ope == "*" || ope == "==" || ope == "!=" || ope == "&&" || ope == "||") {
                //更新活跃信息
                nowActive[getMarkID(mark_1)] = isActive[line][0];
                nowActive[getMarkID(mark_2)] = isActive[line][1];
                nowActive[getMarkID(result)] = isActive[line][2];

                allocateRegister(blockQua[line], 0);

                //如果mark_2在寄存器里，就使用寄存器
                if(useRegister[1] != -1) {
                    mark_2 = "R" + to_string(useRegister[1]);
                }
                //保存式子
                if(ope == "+") {
                    if(useRegister[0] != -1)
                        temp = "ADD\tR" + to_string(useRegister[0]) + "\t," + mark_2;
                    else
                        temp = "ADD\tR" + to_string(useRegister[1]) + "\t," + mark_2;
                }
                else if(ope == "*") {
                    if(useRegister[0] != -1)
                        temp = "MUL\tR" + to_string(useRegister[0]) + "\t," + mark_2;
                    else
                        temp = "MUL\tR" + to_string(useRegister[1]) + "\t," + mark_2;
                }
                else if(ope == "==") {
                    if(useRegister[0] != -1)
                        temp = "EQ\tR" + to_string(useRegister[0]) + "\t," + mark_2;
                    else
                        temp = "EQ\tR" + to_string(useRegister[1]) + "\t," + mark_2;
                }
                else if(ope == "!=") {
                    if(useRegister[0] != -1)
                        temp = "NE\tR" + to_string(useRegister[0]) + "\t," + mark_2;
                    else
                        temp = "NE\tR" + to_string(useRegister[1]) + "\t," + mark_2;
                }
                else if(ope == "||") {
                    if(useRegister[0] != -1)
                        temp = "OR\tR" + to_string(useRegister[0]) + "\t," + mark_2;
                    else
                        temp = "OR\tR" + to_string(useRegister[1]) + "\t," + mark_2;
                }
                else if(ope == "&&") {
                    if(useRegister[0] != -1)
                        temp = "AND\tR" + to_string(useRegister[0]) + "\t," + mark_2;
                    else
                        temp = "AND\tR" + to_string(useRegister[1]) + "\t," + mark_2;
                }
                target.push_back(temp);
            }

            //赋值语句
            else if(ope == "=") {
                //更新活跃信息
                nowActive[getMarkID(mark_1)] = isActive[line][0];
                nowActive[getMarkID(result)] = isActive[line][2];

                allocateRegister(blockQua[line], 0);

                //
                // temp = "ST\tR" + to_string(useRegister[0]) + "\t," + result;
                // target.push_back(temp);
            }
#ifdef DEBUG
            check();
#endif
        }
        //块结束时保存所有寄存器值
        for(int i = 0; i < REG_SIZE; i++) {
            if(mark[RDL[i]].isTempMark() || checkConstKind(mark[RDL[i]].markString))
                continue;
            temp = "ST\tR" + to_string(i) + "\t," + mark[RDL[i]].markString;
            target.push_back(temp);
        }
        clearRegister();
    }
    //重置寄存器状态
    void clearRegister() {
        memset(nowActive, 0, sizeof(nowActive));
        for(int i = 0; i < 4; i++) {
            RDL[i] = 0;
        }
    }
    //输出所有值
    void printTarget() {
        ofstream out(TARGET_PATH);
        for(int i = 0; i < target.size(); i++) {
            out << target[i] << endl;
        }
    }

private:
    //当前活跃信息描述表
    bool nowActive[MARK_SIZE];
    // 寄存器分配值,-1表示不在寄存器中
    int useRegister[3];
    // 寄存器分配函数,分配的结果在 useRegister 中
    void allocateRegister(Quaternary qua, bool canChange = 0) {
        int markID_1, markID_2, markID_3;
        int R_empty, R_Mark_1, R_Mark_2;  // 标志所在寄存器编号

        markID_3 = getMarkID(qua.result);
        R_empty = checkEmptyRegister();  // 空寄存器编号

        markID_1 = getMarkID(qua.mark_1);
        R_Mark_1 = checkInRegister(markID_1);

        if(qua.mark_2 != "_") {
            markID_2 = getMarkID(qua.mark_2);
            R_Mark_2 = checkInRegister(markID_2);
        }

        string temp;
        //不交换,主动释放(mark1)
        if(R_Mark_1 != -1) {
            //活跃的话需要保留值
            if(nowActive[markID_1]) {
                //有空寄存器，移动
                if(R_empty != -1) {
                    temp = "ST\tR" + to_string(R_Mark_1) + "\t,R" + to_string(R_empty);
                    RDL[R_empty] = RDL[R_Mark_1];
                }
                //没有，保存到外部
                else {
                    temp = "ST\tR" + to_string(R_Mark_1) + "\t," + qua.mark_1;
                }
                target.push_back(temp);
            }
            //释放mark_1寄存器
            RDL[R_Mark_1] = markID_3;
            RDL[R_Mark_2] = 0;
            useRegister[0] = R_Mark_1;
            useRegister[1] = R_Mark_2;
            useRegister[2] = R_Mark_1;
        }
        //交换,主动释放(mark2)
        else if(canChange && R_Mark_2 != -1) {
            //活跃的话需要保留值
            if(nowActive[markID_2]) {
                //有空寄存器，移动
                if(R_empty != -1) {
                    temp = "ST\tR" + to_string(R_Mark_2) + "\t,R" + to_string(R_empty);
                    RDL[R_empty] = RDL[R_Mark_2];
                }
                //没有，保存到外部
                else {
                    temp = "ST\tR" + to_string(R_Mark_2) + "\t," + qua.mark_2;
                }
                target.push_back(temp);
            }
            //释放mark_1寄存器
            RDL[R_Mark_2] = markID_3;
            RDL[R_Mark_1] = 0;
            useRegister[0] = R_Mark_2;
            useRegister[1] = R_Mark_1;
            useRegister[2] = R_Mark_2;
        }
        //选空闲者
        else if(R_empty != -1) {
            temp = "LD\tR" + to_string(R_empty) + "\t," + qua.mark_1;
            target.push_back(temp);
            RDL[R_empty] = markID_3;
            useRegister[0] = R_empty;
            useRegister[1] = R_Mark_2;
            useRegister[2] = R_empty;
        }
        //强迫释放(0号寄存器)
        else {
            //活跃的话需要保留值
            if(nowActive[RDL[0]]) {
                //没有空寄存器，保存到外部
                temp = "ST\tR" + to_string(0) + "\t," + mark[RDL[0]].markString;
                target.push_back(temp);
            }
            temp = "LD\tR0\t," + qua.mark_1;
            target.push_back(temp);
            //释放0寄存器
            RDL[0] = markID_3;
            useRegister[0] = 0;
            useRegister[1] = R_Mark_2;
            useRegister[2] = 0;
        }
    }

    // 检测某个标记是否已经在寄存器中，有返回对应下标，否则返回-1
    int checkInRegister(int mark_ID) {
        for(int i = 0; i < REG_SIZE; i++) {
            if(RDL[i] == mark_ID)
                return i;
        }
        return -1;
    }
    // 检测是否有空寄存器中，有返回对应下标，否则返回-1
    int checkEmptyRegister() {
        for(int i = 0; i < REG_SIZE; i++) {
            if(RDL[i] == 0)
                return i;
        }
        return -1;
    }
    //寄存器,内为标记编号
    int RDL[REG_SIZE];
    //生成的目标代码
    vector<string> target;
    //目标代码总行数
    int targetLineNum;
#ifdef DEBUG
    // 调试用文件流
    ofstream debug;
    // 调试用计数器
    int line;
    // 调试用函数
    void check() {
        debug << "########## target building: " << ++line << endl;
        for(int i = 0; i < target.size(); i++) {
            debug << target[i] << endl;
        }
        debug << endl;
        debug << " *** the reg: \t" << endl;
        debug << "\tR0  R1  R2  R3 " << endl;
        for(int i = 0; i < REG_SIZE; i++) {
            debug << "\t" << mark[RDL[i]].markString;
        }
        debug << endl << endl;
    }
#endif
} translater;

//////////////////////////////////分块部分//////////////////////////////

// 基本块类
class Block {
public:
    Block() {
        clear();
        quat.clear();
    }
    //返回条件为真/无条件 时转向的块的ID
    int getNextTrueID() {
        return nextTrueID;
    }
    //返回条件为假时转向的块的ID
    int getNextFalseID() {
        return nextFalseID;
    }
    //重置
    void clear() {
        quat.clear();
        quatCnt = 0;
        nextTrueID = nextFalseID = -1;
    }
    //新加一条四元式
    void addQuaternery(Quaternary qua) {
        quat.push_back(qua);
        quatCnt++;
    }
    //特殊的头四元式
    void addHeadQuaternery(Quaternary header) {
        head = header;
    }
    //设置条件为真/无条件的下个块编号
    void setNextTrueID(int nextTrue) {
        nextTrueID = nextTrue;
    }
    //设置条件为假的下个块编号
    void setNextFalseID(int nextFalse) {
        nextFalseID = nextFalse;
    }
    // DAG优化
    void optimization() {
        DAG.clear();
        for(int i = 0; i < quat.size(); i++) {
            DAG.addEdge(quat[i]);
        }
        quat = DAG.generate();
        quatCnt = quat.size();
    }
    //求解活跃信息
    void getActiveInfo() {
        //遍历所有编号
        for(int i = 0; i < quat.size(); i++) {
            if(quat[i].mark_1 != "_" && checkConstKind(quat[i].mark_1) == NOT_CONST)
                needActive[markID[quat[i].mark_1]] = (quat[i].mark_1[0] != 't');
            if(quat[i].mark_2 != "_" && checkConstKind(quat[i].mark_2) == NOT_CONST)
                needActive[markID[quat[i].mark_2]] = (quat[i].mark_2[0] != 't');
            if(quat[i].result != "_" && checkConstKind(quat[i].result) == NOT_CONST)
                needActive[markID[quat[i].result]] = (quat[i].result[0] != 't');
        }
        //反向遍历所有四元式
        for(int i = quat.size() - 1; i >= 0; i--) {
            if(quat[i].mark_1 != "_" && checkConstKind(quat[i].mark_1) == NOT_CONST) {
                isActive[i][0] = needActive[markID[quat[i].mark_1]];
                needActive[markID[quat[i].mark_1]] = 1;
            }
            else
                isActive[i][0] = 0;
            if(quat[i].mark_2 != "_" && checkConstKind(quat[i].mark_2) == NOT_CONST) {
                isActive[i][1] = needActive[markID[quat[i].mark_2]];
                needActive[markID[quat[i].mark_2]] = 1;
            }
            else
                isActive[i][1] = 0;
            if(quat[i].result != "_" && checkConstKind(quat[i].result) == NOT_CONST) {
                isActive[i][2] = needActive[markID[quat[i].result]];
                needActive[markID[quat[i].result]] = 0;
            }
            else
                isActive[i][2] = 0;
        }
    }
    //块内进行简化与标记活跃信息
    void generate() {
        optimization();
        getActiveInfo();
        // translater.addBlock(quat);
    }
#ifdef DEBUG
    //调试用函数
    void check(ofstream& output) {
        output << " *** after DAG:" << endl;
        if(head.ope.size()) {
            output << "*(" << head.ope << '\t' << head.mark_1 << '\t' << head.mark_2 << '\t' << head.result << '\t' << ')' << endl;
        }
        for(int i = 0; i < quat.size(); i++) {
            output << '(' << quat[i].ope << "\t";
            output << quat[i].mark_1 << "\t";
            output << quat[i].mark_2 << "\t";
            output << quat[i].result << "\t";
            output << ')' << endl;
        }
        output << endl << " *** with activity:" << endl;
        if(head.ope.size()) {
            output << "*(" << head.ope << '\t' << head.mark_1 << '\t' << head.mark_2 << '\t' << head.result << '\t' << ')' << endl;
        }

        for(int i = 0; i < quat.size(); i++) {
            output << '(' << quat[i].ope << "    \t";

            output << quat[i].mark_1;
            if(quat[i].mark_1 != "_" && checkConstKind(quat[i].mark_1) == NOT_CONST)
                output << "(" << isActive[i][0] << ")" << '\t';
            else
                output << "   \t";
            output << quat[i].mark_2;
            if(quat[i].mark_2 != "_" && checkConstKind(quat[i].mark_2) == NOT_CONST)
                output << "(" << isActive[i][1] << ")" << '\t';
            else
                output << "   \t";
            output << quat[i].result;
            if(quat[i].result != "_" && checkConstKind(quat[i].result) == NOT_CONST)
                output << "(" << isActive[i][2] << ")" << '\t';
            else
                output << "   \t";
            output << ')' << endl;
        }
        output << "Ture: " << nextTrueID << "\tFalse: " << nextFalseID << endl;
        output << endl;
    }
#endif
private:
    //特殊的头四元式
    Quaternary head;
    //所有四元式
    vector<Quaternary> quat;
    //四元式记数
    int quatCnt;
    //条件为真/无条件 时转向的块的ID
    int nextTrueID;
    //条件为假时转向的块的ID
    int nextFalseID;

} block[BLOCK_SIZE];

//////////////////////////////////////总交互部分

class Program {
public:
    void readFromFile() {
        ifstream QT("test_input.txt");
        Quaternary que;
        int nowline = 0;
        bool newBlock = 0;
        QT >> que.ope;  //读掉 "QT:"
        while(QT >> que.ope) {
            QT >> que.mark_1 >> que.mark_2 >> que.result;
            allQuats[nowline] = que;

            if(que.ope == "main") {
                block[nowBlock].addHeadQuaternery(que);
                lineBlockID[nowline] = nowBlock;
            }
            else if(que.result != "_") {
                block[nowBlock].addQuaternery(que);
                lineBlockID[nowline] = nowBlock;
            }
            else if(que.ope == "if" || que.ope == "wh" || que.ope == "el" || que.ope == "ie" || que.ope == "do" || que.ope == "we") {
                block[nowBlock].addQuaternery(que);
                lineBlockID[nowline] = nowBlock;
                nowBlock++;
            }
            nowline++;
        }
        quatCnt = nowline;
        nowBlock++;
    }
    //遍历每个块
    void generate() {
        for(int i = 0; i < nowBlock; i++) {
            block[i].generate();
        }
        translater.printTarget();
        check();
    }

private:
    //所有四元式
    Quaternary allQuats[QUAT_SIZE];
    //四元式个数
    int quatCnt;
    //当前已分配块编号
    int nowBlock;
    //某一行四元式对应的块编号
    int lineBlockID[QUAT_SIZE];
#ifdef DEBUG
    //调试用文件流
    ofstream debug;
    //调试用函数
    void check() {
        debug.open(DEBUG_BLOCK_PATH);
        debug << "line's block:" << endl;
        for(int i = 0; i < quatCnt; i++)
            debug << lineBlockID[i] << " ";
        debug << endl << endl;
        for(int i = 0; i < nowBlock; i++) {
            debug << "########### now block ID: " << i << endl;
            block[i].check(debug);
            debug << endl;
        }
    }
#endif
} program;

int main() {
    program.readFromFile();
    program.generate();
    system("pause");
    return 0;
}