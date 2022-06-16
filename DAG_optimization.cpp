/**
 * @Author       : RagnaLP
 * @Date         : 2022-06-13 11:38:40
 * @LastEditTime : 2022-06-16 18:37:05
 * @Description  : 四元式DAG优化程序
 */
#include <iostream>
#include <map>
#include <vector>
using namespace std;
//最大标记数量
const int MARK_SIZE = 10000;
//最大附加标记数量
const int ADD_MARK_SIZE = 100;

//单目运算符符号表
string unaryOperatorTable[1000];
//双目运算符符号表
string binaryOperatorTable[1000];
//常数类型枚举变量，以字节数逐渐增大
enum ConstKind { NOT_CONST, BOOL, CHAR, INT, DOUBLE };
//判断一个标记的常数类型
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
//获取常量计算式计算结果
bool calcConstValue(bool a, bool b, char ope) {
    switch(ope) {
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
//标记数量
int markCnt = 0;
//标记字符串与标记编号的映射
map<string, int> markID;

//标记编号与图中节点编号的映射
int nodeID[MARK_SIZE];
//结点类
class Node {
public:
    //设置主标记
    void setMainMark(int mark_ID) {
        mainMark = mark_ID;
    }
    //设置操作符
    void setMainMark(string ope) {
        opera = ope;
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
    void addAdditionMark(int mark_ID) {
        //如果已经在附加标记中就不再加入
        if(inNode(mark_ID))
            return;
        //当主标记是临时标记但新加入的附加标记不是临时标记时进行交换
        if(mark[mainMark].isTempMark() && !mark[mark_ID].isTempMark()) {
            additionMark.push_back(mainMark);
            mainMark = mark_ID;
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

} node[MARK_SIZE];
//节点数量
int nodeCnt = 0;
//节点有效数量
bool needNode[MARK_SIZE];

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
    node[++nodeCnt].setMainMark(mainMark_ID);
    node[nodeCnt].opera = ope;
    node[nodeCnt].leftNodeID = leftNodeID;
    node[nodeCnt].rightNodeID = rightNodeID;

    mark[mainMark_ID].setNodeID(nodeCnt);
}
//从操作符映射到节点的主要编号，对主要标记进行运算
string getNodeMainMarkString(string markString) {
    // return markString;
    if(markString == "_" || checkConstKind(markString) != NOT_CONST)
        return markString;
    if(markID.find(markString) == markID.end())
        return markString;
    if(mark[markID[markString]].getNodeID() == 0)
        return markString;
    return mark[node[mark[markID[markString]].getNodeID()].getMainMark()].markString;
}
//从输入的四元式构建DAG的一条边
void addEdge(string opera, string mark_1, string mark_2, string result) {
    int markID_1, markID_2, markID_3;
    int nodeID_1, nodeID_2, nodeID_3;
    bool newMark = 0;
    string topMark_1 = getNodeMainMarkString(mark_1);
    string topMark_2 = getNodeMainMarkString(mark_2);
    ConstKind kind_1 = checkConstKind(topMark_1);
    ConstKind kind_2 = checkConstKind(topMark_2);
    if(kind_1 != NOT_CONST && kind_2 != NOT_CONST) {  //常值表达式
        ConstKind resKind = max(kind_1, kind_2);
        string temp;
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
        markID_1 = getMarkID(temp, newMark);
        if(newMark) {  //新建节点
            buildNode(markID_1);
        }

        markID_3 = getMarkID(result, newMark);
        for(int i = 1; i <= nodeCnt; i++) {
            node[i].delAdditionMark(markID_3);
        }
        nodeID_1 = mark[markID_1].getNodeID();
        node[nodeID_1].addAdditionMark(markID_3);
        mark[markID_3].setNodeID(nodeID_1);
    }

    else if(opera == "=") {  //赋值运算
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
            node[nodeID_1].addAdditionMark(markID_3);
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
                        node[i].setMainMark(node[i].additionMark.back());
                        node[i].additionMark.pop_back();
                    }
                    else {
                        node[i].setMainMark(node[i].additionMark[pos]);
                        node[i].additionMark.erase(node[i].additionMark.begin() + pos);
                    }
                }
                else
                    node[i].delAdditionMark(markID_3);
            }
            nodeID_1 = mark[markID_1].getNodeID();
            node[nodeID_1].addAdditionMark(markID_3);
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
            node[sameExpression].addAdditionMark(markID_3);
            mark[markID_3].setNodeID(sameExpression);
        }
        else {
            nodeID_1 = mark[markID_1].getNodeID();
            nodeID_2 = mark[markID_2].getNodeID();
            buildNode(markID_3, nodeID_1, nodeID_2, opera);
        }
    }
}
void check(int line = 0) {
    cout << "####### now finish line:" << line << endl << endl;
    for(int i = nodeCnt; i; i--) {
        cout << "Node id:" << i << endl;
        cout << "l: " << node[i].leftNodeID << "  r:" << node[i].rightNodeID << endl;
        cout << "operator: " << node[i].opera << endl;
        cout << "have: " << mark[node[i].getMainMark()].markString << "|";
        for(int j = 0; j < node[i].additionMark.size(); j++) {
            cout << " " << mark[node[i].additionMark[j]].markString;
        }
        cout << endl << endl;
    }
    for(int i = 0; i < markCnt; i++) {
        cout << mark[i].markString << "\t at \t" << mark[i].getNodeID() << endl;
    }
    cout << "###################" << endl;
    cout << endl;
}
//输出一个四元式,提供的是编号
void printQuaternary(string ope, int markID_1, int markID_2, int resultID) {
    cout << ope << '\t';
    if(markID_1 == -1)
        cout << "_";
    else
        cout << mark[markID_1].markString;
    cout << '\t';
    if(markID_2 == -1)
        cout << "_";
    else
        cout << mark[markID_2].markString;
    cout << '\t';
    cout << mark[resultID].markString << endl;
}
//输出一个四元式,提供的是
void printQuaternary(string ope, string mark_1, string mark_2, string result) {
    cout << ope << '\t' << mark_1 << '\t' << mark_2 << "\t" << result << endl;
}
//重构四元式

void rebuild() {
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
            if(node[i].leftNodeID != 0)
                needNode[node[i].leftNodeID] = 1;
            if(node[i].rightNodeID != 0)
                needNode[node[i].rightNodeID] = 1;
        }
    }
    //再进行输出
    for(int i = 1; i <= nodeCnt; i++) {
        if(needNode[i])
            if(node[i].leftNodeID || node[i].rightNodeID) {
                printQuaternary(node[i].opera, node[node[i].leftNodeID].getMainMark(), node[node[i].rightNodeID].getMainMark(), node[i].getMainMark());
            }
        for(int j = 0; j < node[i].additionMark.size(); j++) {
            if(!mark[node[i].additionMark[j]].isTempMark()) {
                printQuaternary("=", node[i].getMainMark(), -1, node[i].additionMark[j]);
            }
        }
    }
}

int main() {
    freopen("test_input.txt", "r", stdin);
    freopen("test_output.txt", "w", stdout);
    string ope, a, b, c;
    while(cin >> ope) {
        cin >> a >> b >> c;
        addEdge(ope, a, b, c);
    }
    // cout << endl;
    //  check();
    rebuild();
    return 0;
}