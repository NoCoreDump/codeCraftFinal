#include <bits/stdc++.h>
#include <ext/pb_ds/hash_policy.hpp>
#include <ext/pb_ds/assoc_container.hpp>
#include <ext/pb_ds/priority_queue.hpp>
#include <fcntl.h>
#include <unistd.h>

using namespace std;

///@Author 小白兔奶糖
///@Date 2020.5

//TODO: use better heap
//TODO:使用更好的排序、哈希策略(其实也不需要排序了)
//TODO:使用双建图方式可能会有提升
//TODO:完善针对菊花图的优化策略（入0出n）
//TODO:完善针对随机图的优化策略

#define CONFIG_USE_MMAP
//#define TEST

#ifdef CONFIG_USE_MMAP
#include <sys/mman.h>
#include <sys/stat.h>
#endif

///Timer
#ifdef _WIN32   // windows

#include <sysinfoapi.h>

#else   // unix
#include <sys/time.h>
#endif

mutex printLock;

struct UniversalTimer {
    vector<pair<string, int>> logPairs;

    UniversalTimer() {
        setTime();
    }

#ifdef _WIN32
    unsigned int startTime;
    unsigned int endTime;
#else
    struct timeval startTime;
    struct timeval endTime;
#endif

    void setTime() {
#ifdef _WIN32
        startTime = GetTickCount();
#else
        gettimeofday(&startTime, NULL);
#endif
    }

    int getElapsedTimeMS() {
#ifdef _WIN32
        endTime = GetTickCount();
        return int(endTime - startTime);
#else
        gettimeofday(&endTime, NULL);
        return int(1000 * (endTime.tv_sec - startTime.tv_sec) + (endTime.tv_usec - startTime.tv_usec) / 1000);
#endif
    }

    void logTimeImpl(string tag, int elapsedTimeMS) {
        printLock.lock();
        printf("Time consumed for %-20s is %d ms.\n", tag.c_str(), elapsedTimeMS);
        printLock.unlock();
    }

    void logTime(string tag) {
        logTimeImpl(tag, getElapsedTimeMS());
    }

    void resetTimeWithTag(string tag) {
        logPairs.emplace_back(make_pair(tag, getElapsedTimeMS()));
        setTime();
    }

    void printLogs() {
        for (auto &x:logPairs)
            logTimeImpl(x.first, x.second);
    }

    inline void updateProgress(int cur,int tot,bool endFlag){
        if(!endFlag && cur%100!=0) return;
        printLock.lock();
        char buf[64];
        sprintf(buf,"%d/%d (%.2lf%%) [%dms]",cur,tot,cur*100.0/tot,getElapsedTimeMS());
        cout<<"\r"<<buf;
        if(endFlag)
            cout<<endl;
        cout.flush();
        printLock.unlock();
    }

};

#ifdef TEST
UniversalTimer globalTimer;
#endif

typedef uint64_t ull;
typedef uint32_t ui;
//两个点对之间最短路数量不会大于5000
typedef uint16_t us;
typedef double decimal;
#define mp(a, b) ((ull)a<<32 | b)
#define lp(a) ((a>>32) & 0xffffffff)
#define rp(a) (a & 0xffffffff)

//MAXN won't exceed MAXE if only consider u
const int MAXE = 2500000 + 8;
const int MAXN = MAXE * 2;

template <class T, class Alloc=std::allocator<T>>
class ReservedVector
{
public:
    typedef T						value_type;
    typedef value_type*				iterator;
    typedef const value_type*		const_iterator;
    typedef value_type&				reference;
    typedef const T&                const_reference;
    typedef	size_t					size_type;
    typedef ptrdiff_t				difference_type; //distance
protected:
    std::allocator<value_type> _alloc;
    iterator _start;
    iterator _end;
    iterator _end_of_storage; //如果越过会触发扩容
    ui tail;
    value_type *data;
public:
    ReservedVector() {
        data = (T *)malloc(MAXN * sizeof(T));
        _start = data;
        _end = _start;
        tail = 0;
    }
    ReservedVector& operator=(const ReservedVector& rhs) {
        if (this == &rhs)
            return *this;
        size_type n = rhs.cend() - rhs.cbegin();
        _start=_alloc.allocate(n);
        _end = _end_of_storage = std::uninitialized_copy(rhs.cbegin(), rhs.cend(), _start);
    }
    ~ReservedVector() {
        if (_start){
            iterator it(_end);
            while (it != _start)
                _alloc.destroy(--it); //note end is invalid
        }
        _alloc.deallocate(_start, _end_of_storage - _start);
        _start= _end_of_storage = _end = NULL;
    }
    iterator begin() { return _start; }
    iterator end()	 { return _end; }
    const_iterator cbegin() const { return _start; }
    const_iterator cend() const { return _end; }

    size_type size()  { return size_type(_end - _start); }  //cast to size_t
    size_type capacity() { return size_type(_end - _start); }
    inline bool empty() const { return _start == _end; }
    void swap(ReservedVector &other) {
        std::swap(_start, other._start);
        std::swap(_end, other._end);
        std::swap(_end_of_storage, other._end_of_storage);
    }
    reference front() { return *_start; }
    const_reference front() const { return *_start; }
    reference back() { return *(end() - 1); }
    reference operator[] (size_type n) { return *(begin() + n); }
    const_reference top() {
        if (!empty())
            return data[tail];
        else return (T) NULL;
    }
    inline void push_back(const T& value) {
        data[tail++] = value;
        ++_end;
    }
    inline void pop_back() {
        --tail;
        --_end;
    }
    void insert(iterator pos, const value_type & elem) {
        if (pos > end()) {
            *pos = elem;
            _end = pos + 1;
            return;
        }
        iterator p = _end;
        while (p != _start && p != pos){
            *p = *(p-1);
        }
        *pos = elem;
        ++_end;
    }
};

#define NUM_THREAD 8
#define NUM_BLOCK 8192

//线上ID不连续
//id与权重使用ui表示
ui inputs[MAXE * 2]; //u-v pairs
ui weights[MAXE]; //w
__gnu_pbds::gp_hash_table<ui, ui> idHash; //remap sorted id to 1...n
ui tmpIds[MAXN]; //u buffer
ui ids[MAXN]; //sorted u
int inDegreesStart[MAXN];

struct Edge {
    ui to, w;

    Edge() {}

    Edge(ui to, ui w) : to(to), w(w) {}

    bool operator<(const Edge &rhs) const {
        return to < rhs.to;
    }
};

Edge edgesG[MAXE];
Edge edgesGInv[MAXE];

struct Node {
    //record G[st,ed) for Node i
    int st, ed;

    Node() {}

    Node(int st, int ed) : st(st), ed(ed) {}
};

//默认情况下，起止是0
Node G[MAXN]; //u:v-w
Node GInv[MAXN]; //v:u-w

int inDegrees[MAXN];
int outDegrees[MAXN];
ui que[MAXN];

//typedef priority_queue<ull,ReservedVector<ull>,greater<ull>> heap;
typedef priority_queue<ull,std::vector<ull>,greater<ull>> heap;
//typedef priority_queue<PathInfo,ReservedVector<PathInfo>,greater<PathInfo>> heap;
//配对堆（需要对应修改vis数组）
//typedef __gnu_pbds::priority_queue<PathInfo,greater<PathInfo>, __gnu_pbds::pairing_heap_tag> heap;
//Array Based Index Heap
//typedef __gnu_pbds::priority_queue<PathInfo,greater<PathInfo>, __gnu_pbds::binary_heap_tag> heap;

decimal bcSum[MAXN];
int sortedBCIdx[MAXN];

struct Bundle {
    int threadId;

    bool vis[MAXN];
    us pathCnt[MAXN];
    ull dis[MAXN];
    //Betweenness Centrality
    decimal bc[MAXN];
    //C_B(v)=sum(dep_sx[v])
    //dep_sx[v]=pathCnt_sx[v]/pathCnt_sx
    //即s-x最短路上对v的依赖
    decimal dependency[MAXN];
    //predecessor list
    //size won't exceed inDegrees
    ui pList[MAXN];
    //inDegreesSum indicate start idx in pList for i
    //pListEnd indicate end idx in pList for i
    int pListEnd[MAXN];
    ui stack[MAXN];

    heap pq;
    unordered_map<ull,vector<ui>> disMap;
    unordered_set<ull> disSet;

    inline void init(int nodeCnt){
//        priorityQueue.reserve(nodeCnt*2);

//        vis[i] = false;
//        pathCnt[i] = 0;
//        dependency[i] = 0;
//        dis[i] = UINT64_MAX;
//        memset(vis,0,nodeCnt*sizeof(bool));
//        memset(pathCnt,0,nodeCnt*sizeof(int));
//        memset(dependency,0,nodeCnt*sizeof(decimal));
        fill(dis,dis+nodeCnt,UINT64_MAX);
    }

};

Bundle bundles[NUM_THREAD];

//特判入度为0，出度为1的情况
//对于此类点，将计算任务下放至后续结点
ui scaleFactor[MAXN];

int blockNum;
int blockStride;

atomic_int currentBlock(0);
////

int nodeCnt = 0;
int inputSize = 0;
int inputPairSize = 0;

void parseInput(string &testFile) {
#ifndef CONFIG_USE_MMAP
    FILE *file = fopen(testFile.c_str(), "r");
    ui u, v, c;
    while (fscanf(file, "%u,%u,%u", &u, &v, &c) != EOF) {
        inputs[inputSize++] = u;
        inputs[inputSize++] = v;
        weights[inputPairSize++] = c;
    }
#else
    int fd = open(testFile.c_str(), O_RDONLY);
    int fileLength = lseek(fd, 0, SEEK_END);
    char *buffer =
            (char *) mmap(NULL, fileLength, PROT_READ, MAP_PRIVATE, fd, 0);

    //as buffer reaching bufferEnd
    //value won't be in ['0','9'] (end of loop)
    while (*buffer >= '0') {
        ui u = 0, v = 0, w = 0;
        while (*buffer >= '0') {
            u = u * 10 + *(buffer++) - '0';
        }
        ++buffer;
        while (*buffer >= '0') {
            v = v * 10 + *(buffer++) - '0';
        }
        ++buffer;
        while (*buffer >= '0') {
            w = w * 10 + *(buffer++) - '0';
        }
        if (*buffer == '\r') ++buffer;
        ++buffer;
        //skip weight 0
        if(w==0) continue;
        inputs[inputSize++] = u;
        inputs[inputSize++] = v;
        weights[inputPairSize++] = w;
    }
#endif
}

inline void sortInputs(int st, int ed) {
    sort(ids + st, ids + ed);
}

void merge2Result(int st1, int ed1, int st2, int ed2, ui *src, int dstSt, ui *dst) {
    //merge 2 sorted array
    int idx = 0, lim = (ed1 - st1) + (ed2 - st2);
    while (idx < lim) {
        ui mins = UINT32_MAX;
        bool flag = false;
        if (st1 < ed1 && src[st1] < mins) {
            mins = src[st1];
        }
        if (st2 < ed2 && src[st2] < mins) {
            flag = true;
            mins = src[st2];
        }
        dst[dstSt++] = mins;
        if (flag) ++st2;
        else ++st1;
        ++idx;
    }
}

inline void threadingSortInputs(int cnt) {
    //四路排序
    int segmentSize = cnt >> 2;
    int st[4] = {0, segmentSize, segmentSize * 2, segmentSize * 3};
    int ed[4] = {segmentSize, segmentSize * 2, segmentSize * 3, cnt};

    auto t1 = thread(sortInputs, st[0], ed[0]);
    auto t2 = thread(sortInputs, st[1], ed[1]);
    auto t3 = thread(sortInputs, st[2], ed[2]);
    auto t4 = thread(sortInputs, st[3], ed[3]);
    t1.join();
    t2.join();
    t3.join();
    t4.join();

    //4归并至2
    auto t5 = thread(merge2Result, st[0], ed[0], st[1], ed[1], ids, st[0], tmpIds);
    auto t6 = thread(merge2Result, st[2], ed[2], st[3], ed[3], ids, st[2], tmpIds);
    t5.join();
    t6.join();
    //2归并至1
    merge2Result(st[0], ed[1], st[2], ed[3], tmpIds, st[0], ids);
}

void constructGraph() {
#ifdef TEST
    UniversalTimer timerB;
    timerB.setTime();
#endif
    //sort inputs
    int cnt = inputSize;
    memcpy(ids, inputs, sizeof(ui) * cnt);
    if (cnt < 20)
        sort(ids, ids + cnt);
    else threadingSortInputs(cnt);

    nodeCnt = unique(ids, ids + cnt) - ids;
#ifdef TEST
    timerB.resetTimeWithTag("[Construct Graph]-Sort Inputs");
#endif
    //map to [1,nodeCnt)
    nodeCnt++;
    for (int i = 1; i < nodeCnt; ++i) {
        idHash[ids[i - 1]] = i;
    }

#ifdef TEST
    timerB.resetTimeWithTag("[Construct Graph]-Hash");
    printf("%d Nodes in Total\n", nodeCnt - 1);
#endif

    for (int i = 0; i < inputSize; i += 2) {
        ui u = idHash[inputs[i]], v = idHash[inputs[i + 1]];

        inputs[i] = u;
        inputs[i + 1] = v;

        ++inDegrees[v];
        ++outDegrees[u];
    }

#ifdef TEST
    timerB.resetTimeWithTag("[Construct Graph]-Build Graph Remap");
#endif

    int outDegreeCnt = 0;
    int inDegreeCnt = 0;
    for (int i = 1; i < nodeCnt; ++i) {
        G[i] = Node(outDegreeCnt, outDegreeCnt);
        outDegreeCnt += outDegrees[i];
        GInv[i] = Node(inDegreeCnt, inDegreeCnt);
        inDegreeCnt += inDegrees[i];
        inDegreesStart[i+1]=inDegreeCnt;
    }

    for (int i = 0; i < inputSize; i += 2) {
        ui u = inputs[i], v = inputs[i + 1], w = weights[i >> 1];

        edgesG[G[u].ed++] = Edge(v, w);
        edgesGInv[GInv[v].ed++] = Edge(u, w);
    }

#ifdef TEST
    timerB.resetTimeWithTag("[Construct Graph]-Build Graph Add Edge");
    timerB.printLogs();
#endif
}

void topoSort() {
    //remove nodes with inDegree==0
    int queueIdx = -1;
    for (int i = 1; i < nodeCnt; i++) {
        if (0 == inDegrees[i])
            que[++queueIdx] = i;
    }
    while (queueIdx >= 0) {
        ui u = que[queueIdx--];

        for (int idx = G[u].st; idx < G[u].ed; ++idx) {
            ui v = edgesG[idx].to;
            --outDegrees[u];
            if (0 == --inDegrees[v]) {
                que[++queueIdx] = v;
            }
        }
    }

    //remove nodes with outDegree==0
    for (int i = 1; i < nodeCnt; i++) {
        if (0 == outDegrees[i])
            que[++queueIdx] = i;
    }

    while (queueIdx >= 0) {
        int v = que[queueIdx--];
        for (int idx = GInv[v].st; idx < GInv[v].ed; ++idx) {
            ui u = edgesGInv[idx].to;
            --inDegrees[v];
            if (0 == --outDegrees[u]) {
                que[++queueIdx] = u;
            }
        }
    }
}

void sortGraph(int st, int ed) {
    for (int i = st; i < ed; ++i) {
        sort(edgesG + G[i].st, edgesG + G[i].ed);
        sort(edgesGInv + GInv[i].st, edgesGInv + GInv[i].ed);
    }
}

inline void threadingSortGraph() {
    if (nodeCnt < 20) sortGraph(1, nodeCnt);
    else {
        int segmentSize = nodeCnt >> 2;
        int st[4] = {1, segmentSize, segmentSize * 2, segmentSize * 3};
        int ed[4] = {segmentSize, segmentSize * 2, segmentSize * 3, nodeCnt};
        auto t1 = thread(sortGraph, st[0], ed[0]);
        auto t2 = thread(sortGraph, st[1], ed[1]);
        auto t3 = thread(sortGraph, st[2], ed[2]);
        auto t4 = thread(sortGraph, st[3], ed[3]);
        t1.join();
        t2.join();
        t3.join();
        t4.join();
    }
}

void solveBlock(int startBid, int threadId) {
    Bundle &bundle = bundles[threadId];
    int startIdx = startBid * blockStride;
    int endIdx = (startBid + 1) * blockStride;
    endIdx = min(nodeCnt, endIdx);
    if (endIdx <= startIdx) return;

#ifdef TEST
    //   printf("[Thread %d] is solving Block [%d,%d):[%d,%d)\n", threadId, startBid,endBid, startIdx, endIdx);
//    UniversalTimer timer;
//    timer.setTime();
#endif

    bool *vis = bundle.vis;
    us *pathCnt = bundle.pathCnt;
    ull *dis = bundle.dis;
    ui *pList = bundle.pList;
    int *pListEnd = bundle.pListEnd;
    ui *stack = bundle.stack;
    decimal *dependency = bundle.dependency;
    auto &pq=bundle.pq;
    decimal *bc = bundle.bc;
    auto &disMap = bundle.disMap;
    auto &disSet = bundle.disSet;

    for (int i = startIdx; i < endIdx; ++i) {
        if(scaleFactor[i]==0 || outDegrees[i]==0) continue;
        ui head = (ui) i;

#ifdef TEST
        globalTimer.updateProgress(head,nodeCnt-1,head==(ui)nodeCnt-1);
#endif
        ///SSSP Dijkstra
        int stackIdx = 0;
        dis[head] = 0;
        pathCnt[head] = 1;

        disSet.clear();
        auto &pi=disMap[0];
        pi.push_back(head);
        pq.push(0);

        while(!pq.empty()) {

            ull curDis=pq.top();
            auto &idList=disMap[curDis];
            pq.pop();
            if(idList.empty()) continue;
//            cout<<head<<" "<<tmp<<" "<<idList.size()<<endl;
            for(const ui &u:idList){
                if(vis[u]) continue;
                vis[u]=true;
                stack[stackIdx++] = u;
                int lim = G[u].ed;
                for (int it = G[u].st; it < lim; ++it) {
                    ui v = edgesG[it].to;
                    ull disSum=dis[u] + edgesG[it].w;
                    //无符号型不能用减法
                    if (dis[v] > disSum) {
                        dis[v] = disSum;
                        pathCnt[v] = pathCnt[u];
                        pListEnd[v]=inDegreesStart[v];
                        pList[pListEnd[v]++]=u;
                        if(!vis[v]){
                            //相同距离无需多次入队
                            if(disSet.find(dis[v])==disSet.end()){
                                pq.push(dis[v]);
                                disSet.insert(dis[v]);
                            }
                            //通往v的路径可能有多条，故v会出现在多个长度的路径中
                            //每次dis[v]减小会导致v入队
                            //但松弛v时，dis[v]是最小的那个，后续出现的v会被丢弃
                            auto &piv= disMap[dis[v]];
                            piv.push_back(v);
                        }
                    } else if (dis[v] == disSum) {
                        if(pathCnt[u]){
                            pathCnt[v] += pathCnt[u];
                            pList[pListEnd[v]++]=u;
                        }
                    }
                }
            }
            idList.clear();
        }

        vis[head] = false;
        pathCnt[head] = 0;
        dis[head] = UINT64_MAX;

        ///Calc BC
        int scale=scaleFactor[head];
        stackIdx--;
        //stack[0] is always head
        for (int si = stackIdx; si > 0; si--) {
            ui v = stack[si];
            //split it to 2 parts
            //dependency[u] += pathCnt[u] * (dependency[v] + 1) / pathCnt[v] = pathCnt[u] *dv;
            //satisfy pathCnt[v]!=0
            const decimal dv=dependency[v] + 1.0 / pathCnt[v];
            for (int idx=inDegreesStart[v];idx<pListEnd[v];++idx) {
                //u是v的前继，当更新u时，u还在栈中
                //栈中所有元素不重复，栈中的点属于DAG
                //若要判断某条边是否属于DAG，使用dis[v]==dis[u]+edge.w
                //v更新多个u，u也可能被多个v更新
                dependency[pList[idx]] += dv;
            }
//            printf("Head:%d Node:%d Delta:%.3lf\n",head,v,dependency[v]);
            //对于后续涉及的所有结点，更新操作加上scale-1份
            if(scale==1) bc[v] += dependency[v]*pathCnt[v];
            else bc[v] += dependency[v]*pathCnt[v]*scale;

            vis[v] = false;
            pathCnt[v] = 0;
            dependency[v]=0;
            dis[v] = UINT64_MAX;
        }
        //没有使用但是需要清零
        dependency[head]=0;
        if(scale>1){
            //对于当前被下放的结点，要加上其前继个数，累加次数为后续结点数
            //may overflow
            bc[head] +=(scale-1)*stackIdx;
        }
    }

#ifdef TEST
    //    timer.logTime("Current Block");
#endif

}

void solveThread(int threadId) {
#ifdef TEST
    UniversalTimer timer;
    timer.setTime();
#endif
    bundles[threadId].threadId = threadId;
    bundles[threadId].init(nodeCnt);
    while (currentBlock < blockNum) {
        solveBlock(atomic_fetch_add_explicit(&currentBlock, 1, std::memory_order_relaxed), threadId);
    }

#ifdef TEST
    timer.logTime("[Thread " + to_string(threadId) + "].");
#endif
}

void solveWithThreads() {
    if (nodeCnt < 20) {
        blockNum = 1;
        blockStride = nodeCnt;
        solveThread(0);
        memcpy(bcSum, bundles[0].bc, sizeof(decimal) * nodeCnt);
    } else {
        blockNum = min(NUM_BLOCK, nodeCnt);
        blockStride = (nodeCnt + blockNum - 1) / blockNum;
        std::thread threads[NUM_THREAD];
        for (int i = 0; i < NUM_THREAD; ++i) {
            threads[i] = thread(solveThread, i);
        }
        for (auto &thread : threads) {
            thread.join();
        }
        for (int i = 0; i < NUM_THREAD; ++i) {
            auto &bc = bundles[i].bc;
            for (int idx = 0; idx < nodeCnt; ++idx)
                bcSum[idx] += bc[idx];
        }
    }
    for (int i = 0; i < nodeCnt; i++) {
        sortedBCIdx[i] = i;
    }
    auto cmp=[&](int l, int r) {
        if(fabs(bcSum[l] - bcSum[r])<1e-6)
            return l<r;
        return bcSum[l] > bcSum[r];
    };
    sort(sortedBCIdx + 1, sortedBCIdx + nodeCnt, cmp);
}

void save_fwrite(const string &outputFile) {
    FILE *fp = fopen(outputFile.c_str(), "wb");
    char buf[64];
    for (int i = 1; i < min(nodeCnt, 101); i++) {
        //LF for long double
        int idx = sprintf(buf, "%d,%.3lf\n", ids[sortedBCIdx[i] - 1], bcSum[sortedBCIdx[i]]);
        fwrite(buf, idx, sizeof(char), fp);
    }
    fclose(fp);
}

struct Tarjan{
    //在dfs时是第几个被搜到的（init=0)
    int dfn[MAXN];
    //这个点以及其子孙节点连的所有点中dfn最小的值
    int low[MAXN];
    //当前所有可能能构成强连通分量的点
    ui stack[MAXN];
    //sccId==0表示一个点在stack中
    //否则已经出栈
    int sccID[MAXN];
    int sccCnt;
    int nodeNum;
    int stackIdx;

    //map<int,int> sccIDCnt;

    void init(){
        sccCnt=stackIdx=nodeNum=0;
        memset(dfn,0,sizeof(int)*nodeCnt);
        memset(low,0,sizeof(int)*nodeCnt);
        memset(sccID,0,sizeof(int)*nodeCnt);
    }

    void findScc(ui u){
        low[u]=dfn[u]=++nodeNum;
        stack[++stackIdx]=u;
        int st = G[u].st;
        int lim = G[u].ed;
        for (int it = st; it < lim; ++it) {
            ui v = edgesG[it].to;
            if(!dfn[v]){
                findScc(v);
                low[u]=min(low[u],low[v]);
                //if low[v]>=dfn[u] -> 无法通过v及子树返回比u更小结点 -> u是割点
                //if low[v]>dfn[u] -> 无法通过v及子树返回u或u的祖先 -> <u,v>是桥
                //边双连通分量:同一边双内，点与点的边集中无桥
                //通过桥+染色可以求出边bcc
            }else if(!sccID[v]){ //在栈中
                low[u]=min(low[u],dfn[v]);
            }
        }
        if(low[u]==dfn[u]){
            //sccID[u]=++sccCnt;
            ++sccCnt;
            ui cur;
            //update sccID until reaching u
            do{
                cur=stack[stackIdx--];
                sccID[cur]=sccCnt;
                //sccIDCnt[sccCnt]++;
            }while(cur!=u);
        }
    }
};

Tarjan tarjan;

void preprocess(){
    //Data O1
// 入度为0的点713644，
// 其中出度为1的629288，出度为2的50429，出度为3的15835,出度为4的5749
// 若按入度0出度n做拓扑，还可以删去14218个点（713644+14218）
// 若按入度0出度1做拓扑，还可删去2523个点（629288+2523）
//Data O2
// 随机图，SCC=WCC=1

//    for(int i=1;i<nodeCnt;++i){
//        if(!tarjan.dfn[i]){
//            tarjan.findScc(i);
//        }
//    }

    for (int i = 1; i < nodeCnt; i++) {
        if(inDegrees[i]==0 && outDegrees[i]==1){
            scaleFactor[edgesG[G[i].st].to]++;
        }else scaleFactor[i]+=1;
    }
}

int main(int argc, char *argv[]) {

#ifdef TEST
    UniversalTimer timerA, timerB;
#endif

#ifdef CONFIG_USE_MMAP
    //调低进程的友善值
    nice(-20);
#endif

    string testFile;
    if (argc > 1)
        testFile = string(argv[1]);
    else {
#ifdef TEST
        //          testFile = "test_data_SFN.N1560268.E200W.A18875018.txt";
//        testFile = "test_data.final.txt";
//        testFile = "test_iot/data3.txt";
//        testFile = "test_data.final.toy4.txt";
//        testFile = "test_data.final.toy.txt";
//        testFile = "test_data.O1.final.txt";
        testFile = "test_data.O2.final.txt";
//        testFile = "test_data_final1_N1600443_E250W.txt";
//        testFile = "test_data.N111314.E200W.A19630345.txt";
#else
        testFile = "/home/data14.txt";
#endif
    }

    string outputFile = "/codeCraftFinal/result.txt";
#ifdef TEST
    outputFile = "output.txt";
    timerA.setTime();
    timerB.setTime();
#endif

    parseInput(testFile);
#ifdef TEST
    timerB.resetTimeWithTag("Read Input File");
#endif

    constructGraph();
#ifdef TEST
    timerB.resetTimeWithTag("Construct Graph");
#endif

//    topoSort();
//#ifdef TEST
//    timerB.resetTimeWithTag("TopoSort");
//#endif

    threadingSortGraph();

#ifdef TEST
    int edgeCntG = 0, edgeCntGInv = 0;
    for (int i = 1; i < nodeCnt; ++i) edgeCntG += G[i].ed - G[i].st, edgeCntGInv += GInv[i].ed - GInv[i].st;
    printf("[G]-%d [GInv]-%d Edges in Total After Preprocess\n", edgeCntG, edgeCntGInv);
    timerB.resetTimeWithTag("Sort Graph");
    timerB.printLogs();
#endif

    preprocess();
#ifdef TEST
    printf("%d SCC found.\n",tarjan.sccCnt);
    timerB.resetTimeWithTag("Find SCC");
    globalTimer.setTime();
#endif

    solveWithThreads();
#ifdef TEST
    timerB.resetTimeWithTag("Solving Results");
#endif

    save_fwrite(outputFile);

#ifdef TEST
    timerB.resetTimeWithTag("Output");
#endif

#ifdef CF
    usleep(timerA.getElapsedTimeMS()*4*1000);
#endif

#ifdef TEST
    timerB.printLogs();
    timerA.logTime("Whole Process");
#endif

    _exit(0);
    return 0;
}
