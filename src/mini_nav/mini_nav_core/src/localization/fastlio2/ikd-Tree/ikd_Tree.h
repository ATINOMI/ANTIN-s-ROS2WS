/**
 * @file ikd_Tree.h
 * @brief 增量三维 kd-tree 的接口、节点状态、候选最大堆和重建操作环形队列。距离计算使用坐标平方单位，盒范围为左闭右开。
 * @author Antinomy
 * @date 2026-10-05
 */
#pragma once
#include <stdio.h>
#include <queue>
#include <pthread.h>
#include <chrono>
#include <time.h>
#include <unistd.h>
#include <math.h>
#include <algorithm>
#include <memory.h>
#include <pcl/point_types.h>

#define EPSS 1e-6
#define Minimal_Unbalanced_Tree_Size 10
#define Multi_Thread_Rebuild_Point_Num 1500
#define DOWNSAMPLE_SWITCH true
#define ForceRebuildPercentage 0.2
#define Q_LEN 1000000

using namespace std;

// typedef pcl::PointXYZINormal PointType;
// typedef vector<PointType, Eigen::aligned_allocator<PointType>>  PointVector;

/**
 * @brief 三维轴对齐盒，按 [vertex_min,vertex_max) 表示各轴范围。
 * @note 坐标及边长与点云使用同一长度单位，通常为米；不会自动修正上下界。
 */
struct BoxPointType
{
    float vertex_min[3];
    float vertex_max[3];
};

/**
 * @brief 后台重建日志的增删、恢复和懒标记传播操作类型。
 */
enum operation_set
{
    ADD_POINT,
    DELETE_POINT,
    DELETE_BOX,
    ADD_BOX,
    DOWNSAMPLE_DELETE,
    PUSH_DOWN
};

/**
 * @brief 控制 flatten 是否记录普通删除点及记录到哪个线程缓存。
 * @note 体素下采样删除点不会作为普通 removed_points 输出。
 */
enum delete_point_storage_set
{
    NOT_RECORD,
    DELETE_POINTS_REC,
    MULTI_THREAD_REC
};

/**
 * @brief 支持增量插入、懒删除与后台局部重建的三维 kd-tree。
 * @note PointType 须含 x/y/z；内部处理重建同步，但不能据此假定任意公共方法可由多个外部线程同时调用。
 */
template <typename PointType>
class KD_TREE
{
    // using MANUAL_Q_ = MANUAL_Q<typename PointType>;
    // using PointVector = std::vector<PointType>;
    
    // using MANUAL_Q_ = MANUAL_Q<typename PointType>;
public:
    using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
    using Ptr = std::shared_ptr<KD_TREE<PointType>>;
    
    /**
     * @brief 存储分割点、子树计数、空间边界和待传播删除状态。
     * @note TreeSize 包含逻辑删除节点；invalid_point_num 统计全部失效点，down_del_num 单独记录下采样删除。working_flag 与互斥锁不是同一概念。
     */
    struct KD_TREE_NODE
    {
        PointType point;
        int division_axis;
        int TreeSize = 1;
        int invalid_point_num = 0;
        int down_del_num = 0;
        bool point_deleted = false;
        bool tree_deleted = false;
        bool point_downsample_deleted = false;
        bool tree_downsample_deleted = false;
        bool need_push_down_to_left = false;
        bool need_push_down_to_right = false;
        bool working_flag = false;
        pthread_mutex_t push_down_mutex_lock;
        float node_range_x[2], node_range_y[2], node_range_z[2];
        float radius_sq;
        KD_TREE_NODE *left_son_ptr = nullptr;
        KD_TREE_NODE *right_son_ptr = nullptr;
        KD_TREE_NODE *father_ptr = nullptr;
        // For paper data record
        float alpha_del;
        float alpha_bal;
    };

    /**
     * @brief 记录后台快照之后对旧树发生的一次操作。
     * @note op 决定有效负载，点操作读取 point，盒操作读取 boxpoint，PUSH_DOWN 读取删除状态。
     */
    struct Operation_Logger_Type
    {
        PointType point;
        BoxPointType boxpoint;
        bool tree_deleted, tree_downsample_deleted;
        operation_set op;
    };
    // static const PointType zeroP;

    /**
     * @brief 存储近邻候选及其距离平方，用于最大堆排序。
     */
    struct PointType_CMP
    {
        PointType point;
        float dist = 0.0;
        /**
         * @brief 构造近邻候选。
         * @param p 候选点。
         * @param d 到查询点的距离平方，默认正无穷。
         */
        PointType_CMP(PointType p = PointType(), float d = INFINITY)
        {
            this->point = p;
            this->dist = d;
        };
        /**
         * @brief 按距离平方及近似相等时的 x 坐标排序候选。
         * @param a 比较对象。
         * @return 当前距离更小，或距离差<1e-10 且 x 更小时为 true。
         */
        bool operator<(const PointType_CMP &a) const
        {
            if (fabs(dist - a.dist) < 1e-10)
                return point.x < a.point.x;
            else
                return dist < a.dist;
        }
    };

    /**
     * @brief 固定容量候选最大堆，堆顶是当前距离最大的近邻。
     * @note 满容量时 push 直接丢弃；top 不检查空堆，调用者必须先判断 size。
     */
    class MANUAL_HEAP
    {

    public:
        /**
         * @brief 分配指定容量的最大堆。
         * @param max_capacity 可容纳候选数，调用方须保证非负。
         * @throws std::bad_alloc 分配候选数组失败时由 new[] 抛出。
     * @note 拥有动态数组；未定义深拷贝语义，不应按值复制此对象。
         */
        MANUAL_HEAP(int max_capacity = 100)

        {
            cap = max_capacity;
            heap = new PointType_CMP[max_capacity];
            heap_size = 0;
        }

        /**
         * @brief 释放候选数组。
         */
        ~MANUAL_HEAP()
        {
            delete[] heap;
        }
        /**
         * @brief 移除当前首元素。
         * @note 最大堆移除堆顶后下沉；队列推进 head；两者为空时均直接返回。
         */
        void pop()
        {
            if (heap_size == 0)
                return;
            heap[0] = heap[heap_size - 1];
            heap_size--;
            MoveDown(0);
            return;
        }
        /**
         * @brief 读取最大堆堆顶候选。
         * @return 当前最大距离候选副本。
         * @note 必须非空，否则访问未初始化槽位。
         */
        PointType_CMP top()
        {
            return heap[0];
        }
        /**
         * @brief 写入候选或重建操作。
         * @param point 加入最大堆的候选；容量满时丢弃。
         * @note 队列不做溢出检查，且同步由外层调用者持有的锁负责。
         */
        void push(PointType_CMP point)
        {
            if (heap_size >= cap)
                return;
            heap[heap_size] = point;
            FloatUp(heap_size);
            heap_size++;
            return;
        }
        /**
         * @brief 读取容器当前元素数。
         * @return 堆/队列已用元素数；KD_TREE 版本包含逻辑删除节点，根重建繁忙时返回缓存树大小。
         */
        int size()
        {
            return heap_size;
        }
        /**
         * @brief 清空容器逻辑内容。
         * @note 重置计数/索引，不逐个擦除底层存储；队列同时设置 is_empty=true。
         */
        void clear()
        {
            heap_size = 0;
            return;
        }

    private:
        PointType_CMP *heap;
        /**
         * @brief 恢复替换堆顶后的最大堆性质。
         * @param heap_index 从该槽开始比较子节点并下沉。
         */
        void MoveDown(int heap_index)
        {
            int l = heap_index * 2 + 1;
            PointType_CMP tmp = heap[heap_index];
            while (l < heap_size)
            {
                if (l + 1 < heap_size && heap[l] < heap[l + 1])
                    l++;
                if (tmp < heap[l])
                {
                    heap[heap_index] = heap[l];
                    heap_index = l;
                    l = heap_index * 2 + 1;
                }
                else
                    break;
            }
            heap[heap_index] = tmp;
            return;
        }
        /**
         * @brief 恢复插入候选后的最大堆性质。
         * @param heap_index 新候选所在槽，从该槽开始向父节点上浮。
         */
        void FloatUp(int heap_index)
        {
            int ancestor = (heap_index - 1) / 2;
            PointType_CMP tmp = heap[heap_index];
            while (heap_index > 0)
            {
                if (heap[ancestor] < tmp)
                {
                    heap[heap_index] = heap[ancestor];
                    heap_index = ancestor;
                    ancestor = (heap_index - 1) / 2;
                }
                else
                    break;
            }
            heap[heap_index] = tmp;
            return;
        }
        int heap_size = 0;
        int cap = 0;
    };

    /**
     * @brief 固定容量重建操作环形队列。
     * @note 使用前须 clear；push 不检查满队列，front/back 不检查空队列。tail 指向下一写入槽，当前 back 返回 q[tail]，并非最后写入元素。
     */
    class MANUAL_Q
    {
    private:
        int head = 0, tail = 0, counter = 0;
        Operation_Logger_Type q[Q_LEN];
        bool is_empty;

    public:
        /**
         * @brief 移除当前首元素。
         * @note 最大堆移除堆顶后下沉；队列推进 head；两者为空时均直接返回。
         */
        void pop()
        {
            if (counter == 0)
                return;
            head++;
            head %= Q_LEN;
            counter--;
            if (counter == 0)
                is_empty = true;
            return;
        }
        /**
         * @brief 读取队列当前头槽。
         * @return q[head] 的副本。
         * @note 队列必须非空。
         */
        Operation_Logger_Type front()
        {
            return q[head];
        }
        /**
         * @brief 读取队列 tail 所指槽。
         * @return q[tail] 的副本。
         * @note tail 是下一写入位置；此实现不返回最近入队项，不能作为标准 queue::back 使用。
         */
        Operation_Logger_Type back()
        {
            return q[tail];
        }
        /**
         * @brief 清空容器逻辑内容。
         * @note 重置计数/索引，不逐个擦除底层存储；队列同时设置 is_empty=true。
         */
        void clear()
        {
            head = 0;
            tail = 0;
            counter = 0;
            is_empty = true;
            return;
        }
        /**
         * @brief 写入候选或重建操作。
         * @param op 加入环形队列的操作，调用方须确保未满。
         * @note 队列不做溢出检查，且同步由外层调用者持有的锁负责。
         */
        void push(Operation_Logger_Type op)
        {
            q[tail] = op;
            counter++;
            if (is_empty)
                is_empty = false;
            tail++;
            tail %= Q_LEN;
        }
        /**
         * @brief 查询环形队列空标记。
         * @return 最近 clear/pop/push 维护的 is_empty；第一次使用前须 clear。
         */
        bool empty()
        {
            return is_empty;
        }
        /**
         * @brief 读取容器当前元素数。
         * @return 堆/队列已用元素数；KD_TREE 版本包含逻辑删除节点，根重建繁忙时返回缓存树大小。
         */
        int size()
        {
            return counter;
        }
    };

private:
    // Multi-thread Tree Rebuild
    bool termination_flag = false;
    bool rebuild_flag = false;
    pthread_t rebuild_thread;
    pthread_mutex_t termination_flag_mutex_lock, rebuild_ptr_mutex_lock, working_flag_mutex, search_flag_mutex;
    pthread_mutex_t rebuild_logger_mutex_lock, points_deleted_rebuild_mutex_lock;
    // queue<Operation_Logger_Type> Rebuild_Logger;
    MANUAL_Q Rebuild_Logger;
    PointVector Rebuild_PCL_Storage;
    KD_TREE_NODE **Rebuild_Ptr = nullptr;
    int search_mutex_counter = 0;
    /**
     * @brief 适配 pthread 的静态线程入口。
     * @param arg 传入当前 KD_TREE 对象地址。
     * @return 重建循环退出后返回 nullptr。
     */
    static void *multi_thread_ptr(void *arg);
    /**
     * @brief 后台执行子树快照、平衡重建、日志回放和替换。
     * @note 替换前等待近邻查询计数归零，计数 -1 表示禁止新查询；更新需通过日志同步到新树。
     * @note 旧树无有效点时 new_root_node 为 nullptr，当前后续统计仍解引用新根，存在空指针路径。
     */
    void multi_thread_rebuild();
    /**
     * @brief 初始化同步锁并启动后台重建线程。
     * @note 当前未检查 pthread 初始化/创建返回码。
     */
    void start_thread();
    /**
     * @brief 设置终止标记，等待线程退出并销毁同步锁。
     * @note 调用时不能仍有外部树操作使用这些锁。
     */
    void stop_thread();
    /**
     * @brief 将一条并发操作日志应用到重建的新子树。
     * @param root 子树节点或根指针槽；双指针用于允许替换/置空子树。
     * @param operation 待回放的操作日志，op 决定有效负载。
     * @note 回放时禁用再次重建；点操作和 PUSH_DOWN 分支要求可访问的根节点。
     */
    void run_operation(KD_TREE_NODE **root, Operation_Logger_Type operation);
    // KD Tree Functions and augmented variables
    int Treesize_tmp = 0, Validnum_tmp = 0;
    float alpha_bal_tmp = 0.5, alpha_del_tmp = 0.0;
    float delete_criterion_param = 0.5f;
    float balance_criterion_param = 0.7f;
    float downsample_size = 0.2f;
    bool Delete_Storage_Disabled = false;
    KD_TREE_NODE *STATIC_ROOT_NODE = nullptr;
    PointVector Points_deleted;
    PointVector Downsample_Storage;
    PointVector Multithread_Points_deleted;
    /**
     * @brief 初始化新节点的坐标、计数、指针和懒标记互斥锁。
     * @param root 子树节点或根指针槽；双指针用于允许替换/置空子树。
     * @note 只能用于新节点，不能对持锁或仍在使用的节点重复初始化。
     */
    void InitTreeNode(KD_TREE_NODE *root);
    /**
     * @brief 声明节点锁状态诊断接口。
     * @param root 子树节点或根指针槽；双指针用于允许替换/置空子树。
     * @note 本目录未提供定义，不能把此声明当作已实现功能。
     */
    void Test_Lock_States(KD_TREE_NODE *root);
    /**
     * @brief 按最长空间轴的中位点递归建立平衡子树。
     * @param root 子树节点或根指针槽；双指针用于允许替换/置空子树。
     * @param l 构建数组区间起点，含该索引。
     * @param r 构建数组区间终点，含该索引。
     * @param Storage 点数组；公开查询清空，内部遍历/flatten 追加，BuildTree 可重排。
     * @note 输入区间闭合 [l,r]；nth_element 会重排 Storage，l>r 时直接返回。
     */
    void BuildTree(KD_TREE_NODE **root, int l, int r, PointVector &Storage);
    /**
     * @brief 根据子树大小同步重建或提交后台重建候选。
     * @param root 子树节点或根指针槽；双指针用于允许替换/置空子树。
     * @note >=1500 节点尝试登记后台任务，取不到锁时本次不登记；较小子树同步保留有效点并重建。
     */
    void Rebuild(KD_TREE_NODE **root);
    /**
     * @brief 递归逻辑删除盒内有效点并维护重建条件。
     * @param root 子树节点或根指针槽；双指针用于允许替换/置空子树。
     * @param boxpoint 操作/查询盒，各轴为 [min,max)。
     * @param allow_rebuild 是否允许本次递归触发重建；日志回放使用 false。
     * @param is_downsample 是否将失效标记为下采样删除，禁止普通恢复。
     * @return 新删除的有效点数。
     * @note 盒完全覆盖子树时使用懒标记；下采样标记不可被普通盒恢复。部分提前返回路径保留 working_flag=true。
     */
    int Delete_by_range(KD_TREE_NODE **root, BoxPointType boxpoint, bool allow_rebuild, bool is_downsample);
    /**
     * @brief 沿分割轴查找并逻辑删除匹配点。
     * @param root 子树节点或根指针槽；双指针用于允许替换/置空子树。
     * @param point 查询、插入或删除的点；空间比较使用 x/y/z，插入时保存完整点值。
     * @param allow_rebuild 是否允许本次递归触发重建；日志回放使用 false。
     * @note 命中后提前返回，不立即释放节点；部分提前返回路径未复位 working_flag。
     */
    void Delete_by_point(KD_TREE_NODE **root, PointType point, bool allow_rebuild);
    /**
     * @brief 沿分割轴插入新节点并回溯维护统计。
     * @param root 子树节点或根指针槽；双指针用于允许替换/置空子树。
     * @param point 查询、插入或删除的点；空间比较使用 x/y/z，插入时保存完整点值。
     * @param allow_rebuild 是否允许本次递归触发重建；日志回放使用 false。
     * @param father_axis 父节点分割轴，新节点按下一轴循环分割。
     * @note 新节点的分割轴为 (father_axis+1)%3，重建后才重新按最长轴选择。
     */
    void Add_by_point(KD_TREE_NODE **root, PointType point, bool allow_rebuild, int father_axis);
    /**
     * @brief 递归撤销盒内普通删除标记。
     * @param root 子树节点或根指针槽；双指针用于允许替换/置空子树。
     * @param boxpoint 操作/查询盒，各轴为 [min,max)。
     * @param allow_rebuild 是否允许本次递归触发重建；日志回放使用 false。
     * @note 下采样删除状态保留；部分提前返回路径未复位 working_flag。
     */
    void Add_by_range(KD_TREE_NODE **root, BoxPointType boxpoint, bool allow_rebuild);
    /**
     * @brief 通过包围盒距离下界剪枝执行 k 最近邻搜索。
     * @param root 子树节点或根指针槽；双指针用于允许替换/置空子树。
     * @param k_nearest 最大近邻数量，须>0。
     * @param point 查询、插入或删除的点；空间比较使用 x/y/z，插入时保存完整点值。
     * @param q 共享本次查询候选的最大堆。
     * @param max_dist 最大允许距离，使用长度单位，内部比较其平方。
     * @note q 是最大堆，堆顶为最差候选；必须先传播懒标记，以免返回已删除点。
     */
    void Search(KD_TREE_NODE *root, int k_nearest, PointType point, MANUAL_HEAP &q, float max_dist); //priority_queue<PointType_CMP>
    /**
     * @brief 递归收集左闭右开盒内有效点。
     * @param root 子树节点或根指针槽；双指针用于允许替换/置空子树。
     * @param boxpoint 操作/查询盒，各轴为 [min,max)。
     * @param Storage 点数组；公开查询清空，内部遍历/flatten 追加，BuildTree 可重排。
     * @note 向 Storage 追加；完整覆盖子树时通过 flatten 收集。
     */
    void Search_by_range(KD_TREE_NODE *root, BoxPointType boxpoint, PointVector &Storage);
    /**
     * @brief 按子树包围球剪枝收集闭球内有效点。
     * @param root 子树节点或根指针槽；双指针用于允许替换/置空子树。
     * @param point 查询、插入或删除的点；空间比较使用 x/y/z，插入时保存完整点值。
     * @param radius 闭球搜索半径，须非负。
     * @param Storage 点数组；公开查询清空，内部遍历/flatten 追加，BuildTree 可重排。
     * @note 向 Storage 追加；radius 与点坐标同单位。
     */
    void Search_by_radius(KD_TREE_NODE *root, PointType point, float radius, PointVector &Storage);
    /**
     * @brief 按删除比例或子树失衡比例判断是否需要重建。
     * @param root 子树节点或根指针槽；双指针用于允许替换/置空子树。
     * @return 节点数>10 且删除比例超阈值或左右规模比例超界时为 true。
     * @note 要求有效非空节点，阈值比较使用严格大于/小于。
     */
    bool Criterion_Check(KD_TREE_NODE *root);
    /**
     * @brief 将整棵子树的懒删除/恢复标记传播到子节点。
     * @param root 子树节点或根指针槽；双指针用于允许替换/置空子树。
     * @note 下采样删除通过 OR 保留；重建目标子树的传播还需记入日志，不能仅修改旧树。
     */
    void Push_Down(KD_TREE_NODE *root);
    /**
     * @brief 从子节点回算规模、失效计数、边界和父指针。
     * @param root 子树节点或根指针槽；双指针用于允许替换/置空子树。
     * @note 要求非空节点；包围盒按有效点合并，全删时保留几何范围供后续恢复；radius_sq 为盒半对角线长度平方。
     */
    void Update(KD_TREE_NODE *root);
    /**
     * @brief 递归释放子树节点并将根槽置空。
     * @param root 子树节点或根指针槽；双指针用于允许替换/置空子树。
     * @note 释放前下推标记并销毁各节点互斥锁；调用方保证无并发读取。
     */
    void delete_tree_nodes(KD_TREE_NODE **root);
    /**
     * @brief 声明子树下采样接口。
     * @param root 子树节点或根指针槽；双指针用于允许替换/置空子树。
     * @note 本目录未提供定义；实际公开下采样由 Add_Points 实现。
     */
    void downsample(KD_TREE_NODE **root);
    /**
     * @brief 按逐轴绝对差判断两点是否相同。
     * @param a 比较/距离计算的第一个点。
     * @param b 比较/距离计算的第二个点。
     * @return x/y/z 各差值均严格小于 EPSS=1e-6 时为 true。
     * @note 不比较强度、法向量等其他字段。
     */
    bool same_point(PointType a, PointType b);
    /**
     * @brief 计算两点三维欧氏距离平方。
     * @param a 比较/距离计算的第一个点。
     * @param b 比较/距离计算的第二个点。
     * @return dx²+dy²+dz²，单位为点坐标单位的平方。
     */
    float calc_dist(PointType a, PointType b);
    /**
     * @brief 计算查询点到子树包围盒的距离平方下界。
     * @param node 待计算距离下界的子树节点，可为空。
     * @param point 查询、插入或删除的点；空间比较使用 x/y/z，插入时保存完整点值。
     * @return 盒内为 0，盒外按各轴越界量平方求和；node=nullptr 时为正无穷。
     */
    float calc_box_dist(KD_TREE_NODE *node, PointType point);
    /**
     * @brief 按 x 坐标严格排序两点。
     * @param a 比较/距离计算的第一个点。
     * @param b 比较/距离计算的第二个点。
     * @return a.x<b.x。
     */
    static bool point_cmp_x(PointType a, PointType b);
    /**
     * @brief 按 y 坐标严格排序两点。
     * @param a 比较/距离计算的第一个点。
     * @param b 比较/距离计算的第二个点。
     * @return a.y<b.y。
     */
    static bool point_cmp_y(PointType a, PointType b);
    /**
     * @brief 按 z 坐标严格排序两点。
     * @param a 比较/距离计算的第一个点。
     * @param b 比较/距离计算的第二个点。
     * @return a.z<b.z。
     */
    static bool point_cmp_z(PointType a, PointType b);

public:
    /**
     * @brief 设置参数并启动后台重建线程。
     * @param delete_param 删除点比例重建阈值。
     * @param balance_param 子树规模比例重建阈值。
     * @param box_length 体素边长，与点坐标同单位且须>0。
     * @note 构造不建树；插入前须先 Build 非空点云。参数未校验，box_length 必须为正。
     */
    KD_TREE(float delete_param = 0.5, float balance_param = 0.6, float box_length = 0.2);
    /**
     * @brief 停止并等待重建线程，然后释放树节点与缓存。
     * @note 销毁期间不得继续访问或查询该树。
     */
    ~KD_TREE();
    /**
     * @brief 设置触发重建的删除点比例阈值。
     * @param delete_param 删除点比例重建阈值。
     * @note 不验证范围，不立即执行重建。
     */
    void Set_delete_criterion_param(float delete_param)
    {
        delete_criterion_param = delete_param;
    }
    /**
     * @brief 设置触发重建的子树规模比例阈值。
     * @param balance_param 子树规模比例重建阈值。
     * @note 不验证范围，通常应在 (0.5,1) 内，不立即执行重建。
     */
    void Set_balance_criterion_param(float balance_param)
    {
        balance_criterion_param = balance_param;
    }
    /**
     * @brief 设置体素下采样边长。
     * @param downsample_param 新的正体素边长，与点坐标同单位。
     * @note 必须为正，单位与点坐标相同；不重新下采样已有树。
     */
    void set_downsample_param(float downsample_param)
    {
        downsample_size = downsample_param;
    }
    /**
     * @brief 更新删除比例、失衡比例和体素边长参数。
     * @param delete_param 删除点比例重建阈值。
     * @param balance_param 子树规模比例重建阈值。
     * @param box_length 体素边长，与点坐标同单位且须>0。
     * @note 仅调用三个参数 setter，不清空树也不重新启动线程。
     */
    void InitializeKDTree(float delete_param = 0.5, float balance_param = 0.7, float box_length = 0.2);
    /**
     * @brief 读取容器当前元素数。
     * @return 堆/队列已用元素数；KD_TREE 版本包含逻辑删除节点，根重建繁忙时返回缓存树大小。
     */
    int size();
    /**
     * @brief 读取未被逻辑删除的节点数。
     * @return 正常为 TreeSize-invalid_point_num；空树为 0，根重建繁忙且取锁失败为 -1。
     */
    int validnum();
    /**
     * @brief 读取根的失衡比例和删除比例。
     * @param alpha_bal 写入根失衡比例。
     * @param alpha_del 写入根删除比例。
     * @note 要求 Root_Node 非空；取不到重建锁时返回缓存比例，小树比例可能仍为初始化值。
     */
    void root_alpha(float &alpha_bal, float &alpha_del);
    /**
     * @brief 用点云替换当前树并按最长轴中位数建立平衡树。
     * @param point_cloud 初始点云副本，建树时会重排。
     * @note 点云按值传入，可在副本中重排；调用前需确保没有并发重建/访问，空输入仅清除已有实际根。
     */
    void Build(PointVector point_cloud);
    /**
     * @brief 查询指定距离内的至多 k 个有效最近邻。
     * @param point 查询、插入或删除的点；空间比较使用 x/y/z，插入时保存完整点值。
     * @param k_nearest 最大近邻数量，须>0。
     * @param Nearest_Points 输出有效近邻，先清空再按距离升序写入。
     * @param Point_Distance 与近邻一一对应的距离平方，先清空。
     * @param max_dist 最大允许距离，使用长度单位，内部比较其平方。
     * @note k_nearest 必须>0、max_dist 非负；输出按距离平方升序，不保证返回 k 个点。
     */
    void Nearest_Search(PointType point, int k_nearest, PointVector &Nearest_Points, vector<float> &Point_Distance, float max_dist = INFINITY);
    /**
     * @brief 查询左闭右开轴对齐盒中的有效点。
     * @param Box_of_Point 各轴左闭右开的查询盒。
     * @param Storage 点数组；公开查询清空，内部遍历/flatten 追加，BuildTree 可重排。
     * @note 先清空 Storage；范围查询与最近邻采用不同的重建锁路径，不应假设支持任意并发调用。
     */
    void Box_Search(const BoxPointType &Box_of_Point, PointVector &Storage);
    /**
     * @brief 查询闭球内的有效点。
     * @param point 查询、插入或删除的点；空间比较使用 x/y/z，插入时保存完整点值。
     * @param radius 闭球搜索半径，须非负。
     * @param Storage 点数组；公开查询清空，内部遍历/flatten 追加，BuildTree 可重排。
     * @note 先清空 Storage，radius 应非负，边界点满足距离平方<=radius²。
     */
    void Radius_Search(PointType point, const float radius, PointVector &Storage);
    /**
     * @brief 增量添加点，可按体素中心距离选择代表点。
     * @param PointToAdd 输入待插入点数组，函数不修改元素。
     * @param downsample_on 是否结合 DOWNSAMPLE_SWITCH 启用体素代表点筛选。
     * @return 下采样分支实际执行代表点替换/插入的次数；关闭下采样时当前返回 0，不能当作总新增点数。
     * @note 要求非空已建树；同体素保留最靠近中心的一个原始点，不计算质心。
     */
    int Add_Points(PointVector &PointToAdd, bool downsample_on);
    /**
     * @brief 恢复盒内仍保留在树中的普通逻辑删除点。
     * @param BoxPoints 批量盒范围，恢复或删除含义由当前接口决定。
     * @note 不能恢复已重建释放的点，也不能恢复下采样永久删除标记；名称不表示创建新点。
     */
    void Add_Point_Boxes(vector<BoxPointType> &BoxPoints);
    /**
     * @brief 按逐轴 EPSS 容差匹配并逻辑删除输入点。
     * @param PointToDel 待删除点数组，函数不修改元素。
     * @note 不立即释放节点；只比较 x/y/z，不匹配其他点云字段。
     */
    void Delete_Points(PointVector &PointToDel);
    /**
     * @brief 逻辑删除多个左闭右开盒中的有效点。
     * @param BoxPoints 批量盒范围，恢复或删除含义由当前接口决定。
     * @return 本次新删除点总数，重叠盒不会重复计数已经删除的点。
     */
    int Delete_Point_Boxes(vector<BoxPointType> &BoxPoints);
    /**
     * @brief 前序收集有效点，并按模式记录普通删除点。
     * @param root 子树节点或根指针槽；双指针用于允许替换/置空子树。
     * @param Storage 点数组；公开查询清空，内部遍历/flatten 追加，BuildTree 可重排。
     * @param storage_type 删除点缓存记录模式。
     * @note 先下推懒标记，向 Storage 追加而不清空；下采样删除点不进入 removed_points 缓存。
     */
    void flatten(KD_TREE_NODE *root, PointVector &Storage, delete_point_storage_set storage_type);
    /**
     * @brief 提取已由 flatten/重建记录的普通删除点。
     * @param removed_points 追加已记录的普通删除点，不清空原输出。
     * @note 向输出追加并清空两个内部缓存；未被遍历记录的删除点和下采样删除点不会返回。
     */
    void acquire_removed_points(PointVector &removed_points);
    /**
     * @brief 读取根节点维护的轴对齐包围盒。
     * @return 根包围盒；空树或根重建期间取锁失败返回全零盒。
     * @note 全零盒不能用于区分空树、锁繁忙和原点退化点集。
     */
    BoxPointType tree_range();
    PointVector PCL_Storage;
    KD_TREE_NODE *Root_Node = nullptr;
    int max_queue_size = 0;
};

// template <typename PointType>
// PointType KD_TREE<PointType>::zeroP = PointType(0,0,0);
