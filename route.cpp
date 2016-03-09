
#include "route.h"
#include "lib_record.h"
#include <stdio.h>
#include <map>
#include <set>
#include <vector>

//--------------------------------------------------------------------------------------------------------类型定义
typedef std::pair<int, int> Edge;               // 有向边的定义, first代表边的起点， second代表边的终点
typedef std::pair<int, int> EdgeInfo;           // 有向边的附加信息的定义, first代表边的编号， second代表边的权重
typedef std::map<int, std::set<int> > Graph;    // 图的定义
typedef std::map<Edge, EdgeInfo> EdgeInfoDict;  // 有向边附加信息字典的定义
typedef std::set<int> Conditions;               // 必须经过的结点集合
typedef std::pair<int, std::vector<int> > Path; // 路径的定义， 由<权重， 经过的边的有序集合>构成
typedef std::map<std::pair<int, int>, Path> ShortestPathDict;   // 最短路径字典， <<起点， 终点>， 路径>
typedef struct Candidate {                      // Dijstra算法中的候选人
    int nodeNo;
    int pathCost;
    std::vector<int> path;

    bool operator <(const Candidate & other) const {
        // （1）： 结点编号一样， 无论权重大小， 都返回false
        // （2）： 结点编号不一样， 权重大小不一样， 返回权重大小的比较结果
        // （3）： 结点编号不一样， 权重大小一样， 返回结点编号大小比较结果

        if (this->nodeNo == other.nodeNo)
            return false;
        else {
            if(this->pathCost != other.pathCost)
                return this->pathCost < other.pathCost;
            else
                return this->nodeNo < other.nodeNo;
        }
    }
} Candidate;
//--------------------------------------------------------------------------------------------------------数据输入模块函数
int ReadANumberFromStr(char *, int &);                          // 从字符流中读取一个数字
void ReadGraphData(char **, Graph &, EdgeInfoDict &);           // 读取图信息
void ReadConditionsData(char *, int &, int &, Conditions &);    // 读取约束条件信息
//--------------------------------------------------------------------------------------------------------测试函数
void PrintGraph(const Graph &, const EdgeInfoDict &);   //向控制台输出图信息
void PrintConditions(int , int , const Conditions & );  //向控制台输出约束条件信息
void PrintShortestPathDict(const ShortestPathDict & );  //向控制台输出最短路径字典中的信息
//--------------------------------------------------------------------------------------------------------算法函数
void Dijkstra(const Graph &, const EdgeInfoDict &, int, ShortestPathDict &, const std::set<int> & = std::set<int>());    //Dijkstra单源最短路径算法
//--------------------------------------------------------------------------------------------------------赛题入口
void search_route(char *graphStream[5000], int edge_num, char *conditionsStream)
{
    Graph graph;
    EdgeInfoDict edgeInfoDict;
    int source;
    int dest;
    Conditions conditions;
    ShortestPathDict pathDict;


    ReadGraphData(graphStream, graph, edgeInfoDict);
    ReadConditionsData(conditionsStream, source, dest, conditions);

    std::set<int> without;
    without.insert(1);
    without.insert(2);
    Dijkstra(graph, edgeInfoDict, 1, pathDict);

    PrintGraph(graph, edgeInfoDict);
    PrintConditions(source, dest, conditions);
    PrintShortestPathDict(pathDict);
}
//--------------------------------------------------------------------------------------------------------数据输入模块函数实现
int ReadANumberFromStr(char * str, int & index) {
    int res = str[index] - '0';
    while(str[++index] >= '0' && str[index] <= '9') {
        res *= 10;
        res += str[index] - '0';
    }
    ++index;
    return res;
}
void ReadGraphData(char * graphStream[5000], Graph & graph, EdgeInfoDict & edgeInfoDict) {
    for(int i = 0; graphStream[i] != 0x0 && i < 5000; ++i) {
        int j = 0;
        int edgeNo = ReadANumberFromStr(graphStream[i], j);
        int edgeFrom = ReadANumberFromStr(graphStream[i], j);
        int edgeTo = ReadANumberFromStr(graphStream[i], j);
        int edgeCost = ReadANumberFromStr(graphStream[i], j);

        graph[edgeFrom].insert(edgeTo);

        Edge edge(edgeFrom, edgeTo);
        EdgeInfo edgeInfo(edgeNo, edgeCost);

        // 如果边信息字典中已经有了这条边且当前权大于字典中的权， 则不更新字典
        // 否则就要更新字典（可能是插入新边， 也可能是更新旧边）
        if(!(edgeInfoDict.count(edge) && edgeCost > edgeInfoDict[edge].second)) {
            edgeInfoDict[edge] = edgeInfo;
        }
    }
}
void ReadConditionsData(char *conditionsStream, int & source, int & dest, Conditions & conditions) {
    int i = 0;
    source = ReadANumberFromStr(conditionsStream, i);
    dest = ReadANumberFromStr(conditionsStream, i);
    while(conditionsStream[i] != '\0') {
        conditions.insert(ReadANumberFromStr(conditionsStream, i));
    }
}
//--------------------------------------------------------------------------------------------------------测试函数实现
void PrintGraph(const Graph & graph, const EdgeInfoDict & edgeInfoDict) {
    int edgeCount = 0;
    int errorCount = 0;
    printf("------------------------------图信息如下：\n");
    printf("EDGE_NO\tFROM\tTO\tCOST\n");
    for(Graph::const_iterator iter = graph.begin(); iter != graph.end(); ++iter) {
//        Edge edge;
//        edge.first = iter->first;
        int from = iter->first;
        const std::set<int> & toSet = iter->second;
        for(std::set<int>::const_iterator iterInner = toSet.begin(); iterInner != toSet.end(); ++iterInner) {
//            edge.second = *iterInner;
            int to = *iterInner;
            EdgeInfoDict::const_iterator pEdgeInfo = edgeInfoDict.find(Edge(from, to));
            if(pEdgeInfo != edgeInfoDict.end()) {
                int no = (pEdgeInfo->second).first;
                int cost = (pEdgeInfo->second).second;
                printf("%d\t%d\t%d\t%d\n", no, from, to, cost);
                edgeCount++;
            } else {
                printf("发生了一个错误， 一条边的附加信息丢失， 该边为「%d」-->「%d」\n", from, to);
                errorCount++;
            }
//            if(edgeInfoDict.count(edge)) {
//                EdgeInfo edgeInfo = *(edgeInfoDict.find(edge));
//                printf("%d\t%d\t%d\t%d\n", edgeInfo.first, edge.first, edge.second, edgeInfo.second);
//                edgeCount++;
//            } else {
//                printf("发生了一个错误， 一条边的附加信息丢失， 该边为「%d」-->「%d」\n", edge.first, edge.second);
//                errorCount++;
//            }
        }
    }
    printf("图信息输出完毕， 成功输出「%d」条边， 发生了「%d」个错误\n", edgeCount, errorCount);
}
void PrintConditions(int source, int dest, const Conditions & conditions) {
    printf("------------------------------约束条件如下：\n");
    printf("起点： 「%d」， 终点： 「%d」\n", source, dest);
    printf("必须经过的点：");
    bool firstBlood = false;
    for(Conditions::const_iterator iter = conditions.begin(); iter != conditions.end(); ++iter) {
        if(firstBlood)
            printf("|");
        printf("%d", *iter);
        firstBlood = true;
    }
    printf("\n");
}
void PrintShortestPathDict(const ShortestPathDict & pathDict) {
    printf("-------共有%d条最短路径信息-------\n", pathDict.size());
    for(ShortestPathDict::const_iterator iter = pathDict.begin(); iter != pathDict.end(); ++iter) {
        int from = (iter->first).first;
        int to = (iter->first).second;
        int cost = (iter->second).first;
        const std::vector<int> & path = (iter->second).second;
        printf("-------最短路径： 「%d」 --> 「%d」， 路径权重「%d」-------\n", from, to, cost);
        for(std::vector<int>::const_iterator veciter = path.begin(); veciter != path.end(); ++veciter)
            printf("%d ", *veciter);
        printf("\n");
    }
    printf("-------共有%d条最短路径信息-------\n", pathDict.size());
}
//--------------------------------------------------------------------------------------------------------算法函数实现
void Dijkstra(const Graph & graph, const EdgeInfoDict & edgeInfoDict,int source, ShortestPathDict & pathDict,const std::set<int> & withoutPoint) {


    std::set<int> processed;        // 已处理过的结点
    std::set<Candidate> candidates; // 待处理的结点， 配合上Candidate的定义， 这便是一个小顶堆

    // 算法初始化， 起点加入processed集合， 起点的邻接点加入candidates集合
    processed.insert(source);
    Graph::const_iterator pSourceAdjs = graph.find(source);         // 指向graph[source]的迭代器
    if(pSourceAdjs != graph.end()) {                                // 这是一个肯定会满足的条件， 除非source结点不在图中
        const std::set<int> & sourceAdjs = pSourceAdjs->second;
        for(std::set<int>::const_iterator iter = sourceAdjs.begin(); iter != sourceAdjs.end(); ++iter) {
            // 排除必须要排除的点
            if(withoutPoint.count(*iter))
                continue;

            EdgeInfoDict::const_iterator pEdgeInfo = edgeInfoDict.find(Edge(source, *iter));
            if(pEdgeInfo != edgeInfoDict.end()) {
                Candidate candidate;
                candidate.nodeNo = *iter;
                const EdgeInfo & edgeInfo = pEdgeInfo->second;
                candidate.path.push_back(edgeInfo.first);
                candidate.pathCost = edgeInfo.second;
                candidates.insert(candidate);
            }
        }
    }

    // 算法主体开始
    // 第一步： 从候选区挑一个最佳结点， 加入processed集合中去
    // 第二步： 访问最佳结点的所有邻接点， 刷新或扩充候选人集合
    while(!candidates.empty()) {
        // 取出候选区最近的结点, 加入已处理集合中， 并将该结点当前的路径存储到最短路径字典中
        Candidate bestCandidate = *(candidates.begin());
        candidates.erase(bestCandidate);
        processed.insert(bestCandidate.nodeNo);
        pathDict[std::pair<int, int>(source, bestCandidate.nodeNo)] = Path(bestCandidate.pathCost, bestCandidate.path);

        // 访问最佳候选人的所有邻接点， 以刷新或扩充候选结点
        Graph::const_iterator PBestCandidateAdjs = graph.find(bestCandidate.nodeNo);
        // 如果最佳候选人没有邻接点， 直接开始下一轮循环
        if(PBestCandidateAdjs == graph.end())
            continue;

        const std::set<int> & bestCandidateAdjs = PBestCandidateAdjs->second;
        for(std::set<int>::const_iterator iter = bestCandidateAdjs.begin(); iter != bestCandidateAdjs.end(); ++iter) {
            int adjNode = *iter;
            if(processed.count(adjNode) || withoutPoint.count(adjNode))
                continue;

            Candidate candidate;
            candidate.nodeNo = adjNode;
            candidate.path = bestCandidate.path;
            candidate.pathCost = bestCandidate.pathCost;
            EdgeInfoDict::const_iterator PBestCanToCanInfo = edgeInfoDict.find(Edge(bestCandidate.nodeNo, candidate.nodeNo));
            if(PBestCanToCanInfo != edgeInfoDict.end()) {
                const EdgeInfo & edgeBestCanToCanInfo = PBestCanToCanInfo->second;
                candidate.path.push_back(edgeBestCanToCanInfo.first);
                candidate.pathCost += edgeBestCanToCanInfo.second;
            }

            std::set<Candidate>::iterator temp = candidates.find(candidate);
            if(temp == candidates.end() || (*temp).pathCost > candidate.pathCost){
                // 清除原有记录
                candidates.erase(candidate);
                // 更新记录
                candidates.insert(candidate);
            }
        }
    }
}
