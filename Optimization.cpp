/**
 * @Author       : RagnaLP
 * @Date         : 2022-06-13 11:38:40
 * @LastEditTime : 2022-06-17 11:52:40
 * @Description  : 四元式DAG优化程序
 */
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>
using namespace std;
//最大标记数量
const int MARK_SIZE = 1000;
//最大附加标记数量
const int ADD_MARK_SIZE = 100;
//最大四元式数量
const int QUAT_SIZE = 1000;
//最大分块数量
const int BLOCK_SIZE = 1000;

//单目运算符符号表
string unaryOperatorTable[1000];
//双目运算符符号表
string binaryOperatorTable[1000];
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
//输出一个四元式,提供的是字符串
Quaternary buildQuaternary(string ope, string mark_1, string mark_2, string result) {
    Quaternary res;
    res.ope = ope;
    res.mark_1 = mark_1;
    res.mark_2 = mark_2;
    res.result = result;
    return res;
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
    //重构四元式
    vector<Quaternary> rebuild() {
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
            //此时的mark_1会被用来判断块的转向,将其主标记need标志设为1
            markID_1 = getMarkID(mark_1, newMark);
            if(newMark) {  //新建节点
                buildNode(markID_1);
            }
            needNode[mark[markID_1].getNodeID()] = 1;
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
    //获取一个标记的编号,对于第一次出现的标记自动初始化
    int getMarkID(string markString, bool& isNew) {
        if(markID.find(markString) != markID.end()) {  //已有编号
            isNew = 0;
            return markID[markString];
        }
        else {
            isNew = 1;
            //更新标记表
            mark[markCnt].init(markString);
            //分配新的标记编号
            return markID[markString] = markCnt++;
        }
    }
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
} DAG;

//////////////////////////////////分块部分//////////////////////////////
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
        quat = DAG.rebuild();
        quatCnt = quat.size();
    }
    //输出块信息
    void check(ofstream& output) {
        optimization();
        if(head.ope.size()) {
            output << "*(" << head.ope << '\t' << head.mark_1 << '\t' << head.mark_2 << '\t' << head.result << '\t' << ')' << endl;
        }
        for(int i = 0; i < quat.size(); i++) {
            output << '(' << quat[i].ope << '\t' << quat[i].mark_1 << '\t' << quat[i].mark_2 << '\t' << quat[i].result << '\t' << ')' << endl;
        }

        output << "Ture: " << nextTrueID << "\tFalse: " << nextFalseID << endl;
    }

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
    //活动记录
} block[BLOCK_SIZE];

/////////////
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
            else if(que.result != "_" || que.ope == "wh") {
                block[nowBlock].addQuaternery(que);
                lineBlockID[nowline] = nowBlock;
            }
            else if(que.ope == "if" || que.ope == "el" || que.ope == "ie" || que.ope == "do" || que.ope == "we") {
                block[nowBlock].addQuaternery(que);
                lineBlockID[nowline] = nowBlock;
                nowBlock++;
            }
            nowline++;
        }
        quatCnt = nowline;
        nowBlock++;
    }
    void check() {
        ofstream output("test_output.txt");
        for(int i = 0; i < quatCnt; i++)
            output << lineBlockID[i] << " ";
        output << endl;
        for(int i = 0; i < nowBlock; i++) {
            output << "*** now block ID: " << i << endl;
            block[i].check(output);
            output << endl;
        }
        // block[1].check(output);
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
} program;

int main() {
    program.readFromFile();
    program.check();
    system("pause");
    return 0;
}