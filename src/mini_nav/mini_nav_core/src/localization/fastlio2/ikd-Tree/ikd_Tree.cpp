/**
 * @file ikd_Tree.cpp
 * @brief 增量 kd-tree 的建树、懒删除、体素代表点更新、空间检索及后台局部重建。后台重建通过快照和操作日志合并期间的更新。
 * @author Antinomy
 * @date 2026-10-05
 */
#include "ikd_Tree.h"

/*
Description: ikd-Tree: an incremental k-d tree for robotic applications 
Author: Yixi Cai
email: yixicai@connect.hku.hk
*/

/**
 * @brief 设置参数并启动后台重建线程。
 * @param delete_param 删除点比例重建阈值。
 * @param balance_param 子树规模比例重建阈值。
 * @param box_length 体素边长，与点坐标同单位且须>0。
 * @note 构造不建树；插入前须先 Build 非空点云。参数未校验，box_length 必须为正。
 */
template <typename PointType>
KD_TREE<PointType>::KD_TREE(float delete_param, float balance_param, float box_length)
{
    delete_criterion_param = delete_param;
    balance_criterion_param = balance_param;
    downsample_size = box_length;
    Rebuild_Logger.clear();
    termination_flag = false;
    start_thread();
}

/**
 * @brief 停止并等待重建线程，然后释放树节点与缓存。
 * @note 销毁期间不得继续访问或查询该树。
 */
template <typename PointType>
KD_TREE<PointType>::~KD_TREE()
{
    stop_thread();
    Delete_Storage_Disabled = true;
    delete_tree_nodes(&Root_Node);
    PointVector().swap(PCL_Storage);
    Rebuild_Logger.clear();
}



/**
 * @brief 更新删除比例、失衡比例和体素边长参数。
 * @param delete_param 删除点比例重建阈值。
 * @param balance_param 子树规模比例重建阈值。
 * @param box_length 体素边长，与点坐标同单位且须>0。
 * @note 仅调用三个参数 setter，不清空树也不重新启动线程。
 */
template <typename PointType>
void KD_TREE<PointType>::InitializeKDTree(float delete_param, float balance_param, float box_length)
{
    Set_delete_criterion_param(delete_param);
    Set_balance_criterion_param(balance_param);
    set_downsample_param(box_length);
}

/**
 * @brief 初始化新节点的坐标、计数、指针和懒标记互斥锁。
 * @param root 子树节点或根指针槽；双指针用于允许替换/置空子树。
 * @note 只能用于新节点，不能对持锁或仍在使用的节点重复初始化。
 */
template <typename PointType>
void KD_TREE<PointType>::InitTreeNode(KD_TREE_NODE *root)
{
    root->point.x = 0.0f;
    root->point.y = 0.0f;
    root->point.z = 0.0f;
    root->node_range_x[0] = 0.0f;
    root->node_range_x[1] = 0.0f;
    root->node_range_y[0] = 0.0f;
    root->node_range_y[1] = 0.0f;
    root->node_range_z[0] = 0.0f;
    root->node_range_z[1] = 0.0f;
    root->radius_sq = 0.0f;
    root->division_axis = 0;
    root->father_ptr = nullptr;
    root->left_son_ptr = nullptr;
    root->right_son_ptr = nullptr;
    root->TreeSize = 0;
    root->invalid_point_num = 0;
    root->down_del_num = 0;
    root->point_deleted = false;
    root->tree_deleted = false;
    root->need_push_down_to_left = false;
    root->need_push_down_to_right = false;
    root->point_downsample_deleted = false;
    root->working_flag = false;
    pthread_mutex_init(&(root->push_down_mutex_lock), NULL);
}

/**
 * @brief 读取包括逻辑删除节点在内的树大小。
 * @return 当前 TreeSize；空树为 0，根后台重建锁繁忙时返回 Treesize_tmp 缓存。
 */
template <typename PointType>
int KD_TREE<PointType>::size()
{
    int s = 0;
    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node)
    {
        if (Root_Node != nullptr)
        {
            return Root_Node->TreeSize;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        if (!pthread_mutex_trylock(&working_flag_mutex))
        {
            s = Root_Node->TreeSize;
            pthread_mutex_unlock(&working_flag_mutex);
            return s;
        }
        else
        {
            return Treesize_tmp;
        }
    }
}

/**
 * @brief 读取根节点维护的轴对齐包围盒。
 * @return 根包围盒；空树或根重建期间取锁失败返回全零盒。
 * @note 全零盒不能用于区分空树、锁繁忙和原点退化点集。
 */
template <typename PointType>
BoxPointType KD_TREE<PointType>::tree_range()
{
    BoxPointType range;
    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node)
    {
        if (Root_Node != nullptr)
        {
            range.vertex_min[0] = Root_Node->node_range_x[0];
            range.vertex_min[1] = Root_Node->node_range_y[0];
            range.vertex_min[2] = Root_Node->node_range_z[0];
            range.vertex_max[0] = Root_Node->node_range_x[1];
            range.vertex_max[1] = Root_Node->node_range_y[1];
            range.vertex_max[2] = Root_Node->node_range_z[1];
        }
        else
        {
            memset(&range, 0, sizeof(range));
        }
    }
    else
    {
        if (!pthread_mutex_trylock(&working_flag_mutex))
        {
            range.vertex_min[0] = Root_Node->node_range_x[0];
            range.vertex_min[1] = Root_Node->node_range_y[0];
            range.vertex_min[2] = Root_Node->node_range_z[0];
            range.vertex_max[0] = Root_Node->node_range_x[1];
            range.vertex_max[1] = Root_Node->node_range_y[1];
            range.vertex_max[2] = Root_Node->node_range_z[1];
            pthread_mutex_unlock(&working_flag_mutex);
        }
        else
        {
            memset(&range, 0, sizeof(range));
        }
    }
    return range;
}

/**
 * @brief 读取未被逻辑删除的节点数。
 * @return 正常为 TreeSize-invalid_point_num；空树为 0，根重建繁忙且取锁失败为 -1。
 */
template <typename PointType>
int KD_TREE<PointType>::validnum()
{
    int s = 0;
    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node)
    {
        if (Root_Node != nullptr)
            return (Root_Node->TreeSize - Root_Node->invalid_point_num);
        else
            return 0;
    }
    else
    {
        if (!pthread_mutex_trylock(&working_flag_mutex))
        {
            s = Root_Node->TreeSize - Root_Node->invalid_point_num;
            pthread_mutex_unlock(&working_flag_mutex);
            return s;
        }
        else
        {
            return -1;
        }
    }
}

/**
 * @brief 读取根的失衡比例和删除比例。
 * @param alpha_bal 写入根失衡比例。
 * @param alpha_del 写入根删除比例。
 * @note 要求 Root_Node 非空；取不到重建锁时返回缓存比例，小树比例可能仍为初始化值。
 */
template <typename PointType>
void KD_TREE<PointType>::root_alpha(float &alpha_bal, float &alpha_del)
{
    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node)
    {
        alpha_bal = Root_Node->alpha_bal;
        alpha_del = Root_Node->alpha_del;
        return;
    }
    else
    {
        if (!pthread_mutex_trylock(&working_flag_mutex))
        {
            alpha_bal = Root_Node->alpha_bal;
            alpha_del = Root_Node->alpha_del;
            pthread_mutex_unlock(&working_flag_mutex);
            return;
        }
        else
        {
            alpha_bal = alpha_bal_tmp;
            alpha_del = alpha_del_tmp;
            return;
        }
    }
}

/**
 * @brief 初始化同步锁并启动后台重建线程。
 * @note 当前未检查 pthread 初始化/创建返回码。
 */
template <typename PointType>
void KD_TREE<PointType>::start_thread()
{
    pthread_mutex_init(&termination_flag_mutex_lock, NULL);
    pthread_mutex_init(&rebuild_ptr_mutex_lock, NULL);
    pthread_mutex_init(&rebuild_logger_mutex_lock, NULL);
    pthread_mutex_init(&points_deleted_rebuild_mutex_lock, NULL);
    pthread_mutex_init(&working_flag_mutex, NULL);
    pthread_mutex_init(&search_flag_mutex, NULL);
    pthread_create(&rebuild_thread, NULL, multi_thread_ptr, (void *)this);
    printf("Multi thread started \n");
}

/**
 * @brief 设置终止标记，等待线程退出并销毁同步锁。
 * @note 调用时不能仍有外部树操作使用这些锁。
 */
template <typename PointType>
void KD_TREE<PointType>::stop_thread()
{
    pthread_mutex_lock(&termination_flag_mutex_lock);
    termination_flag = true;
    pthread_mutex_unlock(&termination_flag_mutex_lock);
    if (rebuild_thread)
        pthread_join(rebuild_thread, NULL);
    pthread_mutex_destroy(&termination_flag_mutex_lock);
    pthread_mutex_destroy(&rebuild_logger_mutex_lock);
    pthread_mutex_destroy(&rebuild_ptr_mutex_lock);
    pthread_mutex_destroy(&points_deleted_rebuild_mutex_lock);
    pthread_mutex_destroy(&working_flag_mutex);
    pthread_mutex_destroy(&search_flag_mutex);
}

/**
 * @brief 适配 pthread 的静态线程入口。
 * @param arg 传入当前 KD_TREE 对象地址。
 * @return 重建循环退出后返回 nullptr。
 */
template <typename PointType>
void *KD_TREE<PointType>::multi_thread_ptr(void *arg)
{
    KD_TREE *handle = (KD_TREE *)arg;
    handle->multi_thread_rebuild();
    return nullptr;
}

/**
 * @brief 后台执行子树快照、平衡重建、日志回放和替换。
 * @note 替换前等待近邻查询计数归零，计数 -1 表示禁止新查询；更新需通过日志同步到新树。
 * @note 旧树无有效点时 new_root_node 为 nullptr，当前后续统计仍解引用新根，存在空指针路径。
 */
template <typename PointType>
void KD_TREE<PointType>::multi_thread_rebuild()
{
    bool terminated = false;
    KD_TREE_NODE *father_ptr, **new_node_ptr;
    pthread_mutex_lock(&termination_flag_mutex_lock);
    terminated = termination_flag;
    pthread_mutex_unlock(&termination_flag_mutex_lock);
    while (!terminated)
    {
        pthread_mutex_lock(&rebuild_ptr_mutex_lock);
        pthread_mutex_lock(&working_flag_mutex);
        if (Rebuild_Ptr != nullptr)
        {
            /* Step 1：暂停目标子树查询，拍有效点快照。search_mutex_counter=-1 阻止新近邻查询，需在释放快照锁后恢复为 0。 */
            /* Traverse and copy */
            if (!Rebuild_Logger.empty())
            {
                printf("\n\n\n\n\n\n\n\n\n\n\n ERROR!!! \n\n\n\n\n\n\n\n\n");
            }
            rebuild_flag = true;
            if (*Rebuild_Ptr == Root_Node)
            {
                Treesize_tmp = Root_Node->TreeSize;
                Validnum_tmp = Root_Node->TreeSize - Root_Node->invalid_point_num;
                alpha_bal_tmp = Root_Node->alpha_bal;
                alpha_del_tmp = Root_Node->alpha_del;
            }
            KD_TREE_NODE *old_root_node = (*Rebuild_Ptr);
            father_ptr = (*Rebuild_Ptr)->father_ptr;
            PointVector().swap(Rebuild_PCL_Storage);
            // Lock Search
            pthread_mutex_lock(&search_flag_mutex);
            while (search_mutex_counter != 0)
            {
                pthread_mutex_unlock(&search_flag_mutex);
                usleep(1);
                pthread_mutex_lock(&search_flag_mutex);
            }
            search_mutex_counter = -1;
            pthread_mutex_unlock(&search_flag_mutex);
            // Lock deleted points cache
            pthread_mutex_lock(&points_deleted_rebuild_mutex_lock);
            flatten(*Rebuild_Ptr, Rebuild_PCL_Storage, MULTI_THREAD_REC);
            // Unlock deleted points cache
            pthread_mutex_unlock(&points_deleted_rebuild_mutex_lock);
            // Unlock Search
            pthread_mutex_lock(&search_flag_mutex);
            search_mutex_counter = 0;
            pthread_mutex_unlock(&search_flag_mutex);
            pthread_mutex_unlock(&working_flag_mutex);
            /* Step 2：在新树回放快照期间的增删操作。省略日志会使重建替换丢失刚插入的点，或让已删除点再次出现。 */
            /* Rebuild and update missed operations*/
            Operation_Logger_Type Operation;
            KD_TREE_NODE *new_root_node = nullptr;
            if (int(Rebuild_PCL_Storage.size()) > 0)
            {
                BuildTree(&new_root_node, 0, Rebuild_PCL_Storage.size() - 1, Rebuild_PCL_Storage);
                // Rebuild has been done. Updates the blocked operations into the new tree
                pthread_mutex_lock(&working_flag_mutex);
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                int tmp_counter = 0;
                while (!Rebuild_Logger.empty())
                {
                    Operation = Rebuild_Logger.front();
                    max_queue_size = max(max_queue_size, Rebuild_Logger.size());
                    Rebuild_Logger.pop();
                    pthread_mutex_unlock(&rebuild_logger_mutex_lock);
                    pthread_mutex_unlock(&working_flag_mutex);
                    run_operation(&new_root_node, Operation);
                    tmp_counter++;
                    if (tmp_counter % 10 == 0)
                        usleep(1);
                    pthread_mutex_lock(&working_flag_mutex);
                    pthread_mutex_lock(&rebuild_logger_mutex_lock);
                }
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
            /* Step 3：等待查询退出后替换父节点槽位，并更新父链统计；旧节点须在替换和查询屏障完成后才释放。 */
            /* Replace to original tree*/
            // pthread_mutex_lock(&working_flag_mutex);
            pthread_mutex_lock(&search_flag_mutex);
            while (search_mutex_counter != 0)
            {
                pthread_mutex_unlock(&search_flag_mutex);
                usleep(1);
                pthread_mutex_lock(&search_flag_mutex);
            }
            search_mutex_counter = -1;
            pthread_mutex_unlock(&search_flag_mutex);
            if (father_ptr->left_son_ptr == *Rebuild_Ptr)
            {
                father_ptr->left_son_ptr = new_root_node;
            }
            else if (father_ptr->right_son_ptr == *Rebuild_Ptr)
            {
                father_ptr->right_son_ptr = new_root_node;
            }
            else
            {
                throw "Error: Father ptr incompatible with current node\n";
            }
            if (new_root_node != nullptr)
                new_root_node->father_ptr = father_ptr;
            (*Rebuild_Ptr) = new_root_node;
            int valid_old = old_root_node->TreeSize - old_root_node->invalid_point_num;
            /* new_root_node 在快照没有有效点时仍可能为空；当前下一句未检查便解引用，这是现有边界问题。 */
            int valid_new = new_root_node->TreeSize - new_root_node->invalid_point_num;
            if (father_ptr == STATIC_ROOT_NODE)
                Root_Node = STATIC_ROOT_NODE->left_son_ptr;
            KD_TREE_NODE *update_root = *Rebuild_Ptr;
            while (update_root != nullptr && update_root != Root_Node)
            {
                update_root = update_root->father_ptr;
                if (update_root->working_flag)
                    break;
                if (update_root == update_root->father_ptr->left_son_ptr && update_root->father_ptr->need_push_down_to_left)
                    break;
                if (update_root == update_root->father_ptr->right_son_ptr && update_root->father_ptr->need_push_down_to_right)
                    break;
                Update(update_root);
            }
            pthread_mutex_lock(&search_flag_mutex);
            search_mutex_counter = 0;
            pthread_mutex_unlock(&search_flag_mutex);
            Rebuild_Ptr = nullptr;
            pthread_mutex_unlock(&working_flag_mutex);
            rebuild_flag = false;
            /* Delete discarded tree nodes */
            delete_tree_nodes(&old_root_node);
        }
        else
        {
            pthread_mutex_unlock(&working_flag_mutex);
        }
        pthread_mutex_unlock(&rebuild_ptr_mutex_lock);
        pthread_mutex_lock(&termination_flag_mutex_lock);
        terminated = termination_flag;
        pthread_mutex_unlock(&termination_flag_mutex_lock);
        usleep(100);
    }
    printf("Rebuild thread terminated normally\n");
}

/**
 * @brief 将一条并发操作日志应用到重建的新子树。
 * @param root 子树节点或根指针槽；双指针用于允许替换/置空子树。
 * @param operation 待回放的操作日志，op 决定有效负载。
 * @note 回放时禁用再次重建；点操作和 PUSH_DOWN 分支要求可访问的根节点。
 */
template <typename PointType>
void KD_TREE<PointType>::run_operation(KD_TREE_NODE **root, Operation_Logger_Type operation)
{
    switch (operation.op)
    {
    case ADD_POINT:
        Add_by_point(root, operation.point, false, (*root)->division_axis);
        break;
    case ADD_BOX:
        Add_by_range(root, operation.boxpoint, false);
        break;
    case DELETE_POINT:
        Delete_by_point(root, operation.point, false);
        break;
    case DELETE_BOX:
        Delete_by_range(root, operation.boxpoint, false, false);
        break;
    case DOWNSAMPLE_DELETE:
        Delete_by_range(root, operation.boxpoint, false, true);
        break;
    case PUSH_DOWN:
        (*root)->tree_downsample_deleted |= operation.tree_downsample_deleted;
        (*root)->point_downsample_deleted |= operation.tree_downsample_deleted;
        (*root)->tree_deleted = operation.tree_deleted || (*root)->tree_downsample_deleted;
        (*root)->point_deleted = (*root)->tree_deleted || (*root)->point_downsample_deleted;
        if (operation.tree_downsample_deleted)
            (*root)->down_del_num = (*root)->TreeSize;
        if (operation.tree_deleted)
            (*root)->invalid_point_num = (*root)->TreeSize;
        else
            (*root)->invalid_point_num = (*root)->down_del_num;
        (*root)->need_push_down_to_left = true;
        (*root)->need_push_down_to_right = true;
        break;
    default:
        break;
    }
}

/**
 * @brief 用点云替换当前树并按最长轴中位数建立平衡树。
 * @param point_cloud 初始点云副本，建树时会重排。
 * @note 点云按值传入，可在副本中重排；调用前需确保没有并发重建/访问，空输入仅清除已有实际根。
 */
template <typename PointType>
void KD_TREE<PointType>::Build(PointVector point_cloud)
{
    if (Root_Node != nullptr)
    {
        delete_tree_nodes(&Root_Node);
    }
    if (point_cloud.size() == 0)
        return;
    STATIC_ROOT_NODE = new KD_TREE_NODE;
    InitTreeNode(STATIC_ROOT_NODE);
    BuildTree(&STATIC_ROOT_NODE->left_son_ptr, 0, point_cloud.size() - 1, point_cloud);
    Update(STATIC_ROOT_NODE);
    STATIC_ROOT_NODE->TreeSize = 0;
    Root_Node = STATIC_ROOT_NODE->left_son_ptr;
}

/**
 * @brief 查询指定距离内的至多 k 个有效最近邻。
 * @param point 查询、插入或删除的点；空间比较使用 x/y/z，插入时保存完整点值。
 * @param k_nearest 最大近邻数量，须>0。
 * @param Nearest_Points 输出有效近邻，先清空再按距离升序写入。
 * @param Point_Distance 与近邻一一对应的距离平方，先清空。
 * @param max_dist 最大允许距离，使用长度单位，内部比较其平方。
 * @note k_nearest 必须>0、max_dist 非负；输出按距离平方升序，不保证返回 k 个点。
 */
template <typename PointType>
void KD_TREE<PointType>::Nearest_Search(PointType point, int k_nearest, PointVector &Nearest_Points, vector<float> &Point_Distance, float max_dist)
{
    /* 最大堆维护至多 k 个候选；随后从最大值逆序插到输出头部，得到距离平方升序，不能将输出误读成距离。 */
    MANUAL_HEAP q(2 * k_nearest);
    q.clear();
    vector<float>().swap(Point_Distance);
    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node)
    {
        Search(Root_Node, k_nearest, point, q, max_dist);
    }
    else
    {
        pthread_mutex_lock(&search_flag_mutex);
        while (search_mutex_counter == -1)
        {
            pthread_mutex_unlock(&search_flag_mutex);
            usleep(1);
            pthread_mutex_lock(&search_flag_mutex);
        }
        search_mutex_counter += 1;
        pthread_mutex_unlock(&search_flag_mutex);
        Search(Root_Node, k_nearest, point, q, max_dist);
        pthread_mutex_lock(&search_flag_mutex);
        search_mutex_counter -= 1;
        pthread_mutex_unlock(&search_flag_mutex);
    }
    int k_found = min(k_nearest, int(q.size()));
    PointVector().swap(Nearest_Points);
    vector<float>().swap(Point_Distance);
    for (int i = 0; i < k_found; i++)
    {
        Nearest_Points.insert(Nearest_Points.begin(), q.top().point);
        Point_Distance.insert(Point_Distance.begin(), q.top().dist);
        q.pop();
    }
    return;
}

/**
 * @brief 查询左闭右开轴对齐盒中的有效点。
 * @param Box_of_Point 各轴左闭右开的查询盒。
 * @param Storage 点数组；公开查询清空，内部遍历/flatten 追加，BuildTree 可重排。
 * @note 先清空 Storage；范围查询与最近邻采用不同的重建锁路径，不应假设支持任意并发调用。
 */
template <typename PointType>
void KD_TREE<PointType>::Box_Search(const BoxPointType &Box_of_Point, PointVector &Storage)
{
    Storage.clear();
    Search_by_range(Root_Node, Box_of_Point, Storage);
}

/**
 * @brief 查询闭球内的有效点。
 * @param point 查询、插入或删除的点；空间比较使用 x/y/z，插入时保存完整点值。
 * @param radius 闭球搜索半径，须非负。
 * @param Storage 点数组；公开查询清空，内部遍历/flatten 追加，BuildTree 可重排。
 * @note 先清空 Storage，radius 应非负，边界点满足距离平方<=radius²。
 */
template <typename PointType>
void KD_TREE<PointType>::Radius_Search(PointType point, const float radius, PointVector &Storage)
{
    Storage.clear();
    Search_by_radius(Root_Node, point, radius, Storage);
}

/**
 * @brief 增量添加点，可按体素中心距离选择代表点。
 * @param PointToAdd 输入待插入点数组，函数不修改元素。
 * @param downsample_on 是否结合 DOWNSAMPLE_SWITCH 启用体素代表点筛选。
 * @return 下采样分支实际执行代表点替换/插入的次数；关闭下采样时当前返回 0，不能当作总新增点数。
 * @note 要求非空已建树；同体素保留最靠近中心的一个原始点，不计算质心。
 */
template <typename PointType>
int KD_TREE<PointType>::Add_Points(PointVector &PointToAdd, bool downsample_on)
{
    int NewPointSize = PointToAdd.size();
    int tree_size = size();
    BoxPointType Box_of_Point;
    PointType downsample_result, mid_point;
    /* 每个体素从已有点和新点中选择最靠近中心的原始点；普通删除可恢复，下采样删除标记永久保留直至节点被释放。 */
    bool downsample_switch = downsample_on && DOWNSAMPLE_SWITCH;
    float min_dist, tmp_dist;
    int tmp_counter = 0;
    for (int i = 0; i < PointToAdd.size(); i++)
    {
        if (downsample_switch)
        {
            Box_of_Point.vertex_min[0] = floor(PointToAdd[i].x / downsample_size) * downsample_size;
            Box_of_Point.vertex_max[0] = Box_of_Point.vertex_min[0] + downsample_size;
            Box_of_Point.vertex_min[1] = floor(PointToAdd[i].y / downsample_size) * downsample_size;
            Box_of_Point.vertex_max[1] = Box_of_Point.vertex_min[1] + downsample_size;
            Box_of_Point.vertex_min[2] = floor(PointToAdd[i].z / downsample_size) * downsample_size;
            Box_of_Point.vertex_max[2] = Box_of_Point.vertex_min[2] + downsample_size;
            mid_point.x = Box_of_Point.vertex_min[0] + (Box_of_Point.vertex_max[0] - Box_of_Point.vertex_min[0]) / 2.0;
            mid_point.y = Box_of_Point.vertex_min[1] + (Box_of_Point.vertex_max[1] - Box_of_Point.vertex_min[1]) / 2.0;
            mid_point.z = Box_of_Point.vertex_min[2] + (Box_of_Point.vertex_max[2] - Box_of_Point.vertex_min[2]) / 2.0;
            PointVector().swap(Downsample_Storage);
            Search_by_range(Root_Node, Box_of_Point, Downsample_Storage);
            min_dist = calc_dist(PointToAdd[i], mid_point);
            downsample_result = PointToAdd[i];
            for (int index = 0; index < Downsample_Storage.size(); index++)
            {
                tmp_dist = calc_dist(Downsample_Storage[index], mid_point);
                if (tmp_dist < min_dist)
                {
                    min_dist = tmp_dist;
                    downsample_result = Downsample_Storage[index];
                }
            }
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node)
            {
                if (Downsample_Storage.size() > 1 || same_point(PointToAdd[i], downsample_result))
                {
                    if (Downsample_Storage.size() > 0)
                        Delete_by_range(&Root_Node, Box_of_Point, true, true);
                    Add_by_point(&Root_Node, downsample_result, true, Root_Node->division_axis);
                    tmp_counter++;
                }
            }
            else
            {
                if (Downsample_Storage.size() > 1 || same_point(PointToAdd[i], downsample_result))
                {
                    Operation_Logger_Type operation_delete, operation;
                    operation_delete.boxpoint = Box_of_Point;
                    operation_delete.op = DOWNSAMPLE_DELETE;
                    operation.point = downsample_result;
                    operation.op = ADD_POINT;
                    pthread_mutex_lock(&working_flag_mutex);
                    if (Downsample_Storage.size() > 0)
                        Delete_by_range(&Root_Node, Box_of_Point, false, true);
                    Add_by_point(&Root_Node, downsample_result, false, Root_Node->division_axis);
                    tmp_counter++;
                    if (rebuild_flag)
                    {
                        pthread_mutex_lock(&rebuild_logger_mutex_lock);
                        if (Downsample_Storage.size() > 0)
                            Rebuild_Logger.push(operation_delete);
                        Rebuild_Logger.push(operation);
                        pthread_mutex_unlock(&rebuild_logger_mutex_lock);
                    }
                    pthread_mutex_unlock(&working_flag_mutex);
                };
            }
        }
        else
        {
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node)
            {
                Add_by_point(&Root_Node, PointToAdd[i], true, Root_Node->division_axis);
            }
            else
            {
                Operation_Logger_Type operation;
                operation.point = PointToAdd[i];
                operation.op = ADD_POINT;
                pthread_mutex_lock(&working_flag_mutex);
                Add_by_point(&Root_Node, PointToAdd[i], false, Root_Node->division_axis);
                if (rebuild_flag)
                {
                    pthread_mutex_lock(&rebuild_logger_mutex_lock);
                    Rebuild_Logger.push(operation);
                    pthread_mutex_unlock(&rebuild_logger_mutex_lock);
                }
                pthread_mutex_unlock(&working_flag_mutex);
            }
        }
    }
    return tmp_counter;
}

/**
 * @brief 恢复盒内仍保留在树中的普通逻辑删除点。
 * @param BoxPoints 批量盒范围，恢复或删除含义由当前接口决定。
 * @note 不能恢复已重建释放的点，也不能恢复下采样永久删除标记；名称不表示创建新点。
 */
template <typename PointType>
void KD_TREE<PointType>::Add_Point_Boxes(vector<BoxPointType> &BoxPoints)
{
    for (int i = 0; i < BoxPoints.size(); i++)
    {
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node)
        {
            Add_by_range(&Root_Node, BoxPoints[i], true);
        }
        else
        {
            Operation_Logger_Type operation;
            operation.boxpoint = BoxPoints[i];
            operation.op = ADD_BOX;
            pthread_mutex_lock(&working_flag_mutex);
            Add_by_range(&Root_Node, BoxPoints[i], false);
            if (rebuild_flag)
            {
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(operation);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
            pthread_mutex_unlock(&working_flag_mutex);
        }
    }
    return;
}

/**
 * @brief 按逐轴 EPSS 容差匹配并逻辑删除输入点。
 * @param PointToDel 待删除点数组，函数不修改元素。
 * @note 不立即释放节点；只比较 x/y/z，不匹配其他点云字段。
 */
template <typename PointType>
void KD_TREE<PointType>::Delete_Points(PointVector &PointToDel)
{
    for (int i = 0; i < PointToDel.size(); i++)
    {
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node)
        {
            Delete_by_point(&Root_Node, PointToDel[i], true);
        }
        else
        {
            Operation_Logger_Type operation;
            operation.point = PointToDel[i];
            operation.op = DELETE_POINT;
            pthread_mutex_lock(&working_flag_mutex);
            Delete_by_point(&Root_Node, PointToDel[i], false);
            if (rebuild_flag)
            {
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(operation);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
            pthread_mutex_unlock(&working_flag_mutex);
        }
    }
    return;
}

/**
 * @brief 逻辑删除多个左闭右开盒中的有效点。
 * @param BoxPoints 批量盒范围，恢复或删除含义由当前接口决定。
 * @return 本次新删除点总数，重叠盒不会重复计数已经删除的点。
 */
template <typename PointType>
int KD_TREE<PointType>::Delete_Point_Boxes(vector<BoxPointType> &BoxPoints)
{
    int tmp_counter = 0;
    for (int i = 0; i < BoxPoints.size(); i++)
    {
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node)
        {
            tmp_counter += Delete_by_range(&Root_Node, BoxPoints[i], true, false);
        }
        else
        {
            Operation_Logger_Type operation;
            operation.boxpoint = BoxPoints[i];
            operation.op = DELETE_BOX;
            pthread_mutex_lock(&working_flag_mutex);
            tmp_counter += Delete_by_range(&Root_Node, BoxPoints[i], false, false);
            if (rebuild_flag)
            {
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(operation);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
            pthread_mutex_unlock(&working_flag_mutex);
        }
    }
    return tmp_counter;
}

/**
 * @brief 提取已由 flatten/重建记录的普通删除点。
 * @param removed_points 追加已记录的普通删除点，不清空原输出。
 * @note 向输出追加并清空两个内部缓存；未被遍历记录的删除点和下采样删除点不会返回。
 */
template <typename PointType>
void KD_TREE<PointType>::acquire_removed_points(PointVector &removed_points)
{
    pthread_mutex_lock(&points_deleted_rebuild_mutex_lock);
    for (int i = 0; i < Points_deleted.size(); i++)
    {
        removed_points.push_back(Points_deleted[i]);
    }
    for (int i = 0; i < Multithread_Points_deleted.size(); i++)
    {
        removed_points.push_back(Multithread_Points_deleted[i]);
    }
    Points_deleted.clear();
    Multithread_Points_deleted.clear();
    pthread_mutex_unlock(&points_deleted_rebuild_mutex_lock);
    return;
}

/**
 * @brief 按最长空间轴的中位点递归建立平衡子树。
 * @param root 子树节点或根指针槽；双指针用于允许替换/置空子树。
 * @param l 构建数组区间起点，含该索引。
 * @param r 构建数组区间终点，含该索引。
 * @param Storage 点数组；公开查询清空，内部遍历/flatten 追加，BuildTree 可重排。
 * @note 输入区间闭合 [l,r]；nth_element 会重排 Storage，l>r 时直接返回。
 */
template <typename PointType>
void KD_TREE<PointType>::BuildTree(KD_TREE_NODE **root, int l, int r, PointVector &Storage)
{
    if (l > r)
        return;
    *root = new KD_TREE_NODE;
    InitTreeNode(*root);
    int mid = (l + r) >> 1;
    int div_axis = 0;
    int i;
    // Find the best division Axis
    float min_value[3] = {INFINITY, INFINITY, INFINITY};
    float max_value[3] = {-INFINITY, -INFINITY, -INFINITY};
    float dim_range[3] = {0, 0, 0};
    for (i = l; i <= r; i++)
    {
        min_value[0] = min(min_value[0], Storage[i].x);
        min_value[1] = min(min_value[1], Storage[i].y);
        min_value[2] = min(min_value[2], Storage[i].z);
        max_value[0] = max(max_value[0], Storage[i].x);
        max_value[1] = max(max_value[1], Storage[i].y);
        max_value[2] = max(max_value[2], Storage[i].z);
    }
    // Select the longest dimension as division axis
    for (i = 0; i < 3; i++)
        dim_range[i] = max_value[i] - min_value[i];
    for (i = 1; i < 3; i++)
        if (dim_range[i] > dim_range[div_axis])
            div_axis = i;
    // Divide by the division axis and recursively build.

    (*root)->division_axis = div_axis;
    switch (div_axis)
    {
    case 0:
        nth_element(begin(Storage) + l, begin(Storage) + mid, begin(Storage) + r + 1, point_cmp_x);
        break;
    case 1:
        nth_element(begin(Storage) + l, begin(Storage) + mid, begin(Storage) + r + 1, point_cmp_y);
        break;
    case 2:
        nth_element(begin(Storage) + l, begin(Storage) + mid, begin(Storage) + r + 1, point_cmp_z);
        break;
    default:
        nth_element(begin(Storage) + l, begin(Storage) + mid, begin(Storage) + r + 1, point_cmp_x);
        break;
    }
    (*root)->point = Storage[mid];
    KD_TREE_NODE *left_son = nullptr, *right_son = nullptr;
    BuildTree(&left_son, l, mid - 1, Storage);
    BuildTree(&right_son, mid + 1, r, Storage);
    (*root)->left_son_ptr = left_son;
    (*root)->right_son_ptr = right_son;
    Update((*root));
    return;
}

/**
 * @brief 根据子树大小同步重建或提交后台重建候选。
 * @param root 子树节点或根指针槽；双指针用于允许替换/置空子树。
 * @note >=1500 节点尝试登记后台任务，取不到锁时本次不登记；较小子树同步保留有效点并重建。
 */
template <typename PointType>
void KD_TREE<PointType>::Rebuild(KD_TREE_NODE **root)
{
    KD_TREE_NODE *father_ptr;
    if ((*root)->TreeSize >= Multi_Thread_Rebuild_Point_Num)
    {
        if (!pthread_mutex_trylock(&rebuild_ptr_mutex_lock))
        {
            if (Rebuild_Ptr == nullptr || ((*root)->TreeSize > (*Rebuild_Ptr)->TreeSize))
            {
                Rebuild_Ptr = root;
            }
            pthread_mutex_unlock(&rebuild_ptr_mutex_lock);
        }
    }
    else
    {
        father_ptr = (*root)->father_ptr;
        int size_rec = (*root)->TreeSize;
        PCL_Storage.clear();
        flatten(*root, PCL_Storage, DELETE_POINTS_REC);
        delete_tree_nodes(root);
        BuildTree(root, 0, PCL_Storage.size() - 1, PCL_Storage);
        if (*root != nullptr)
            (*root)->father_ptr = father_ptr;
        if (*root == Root_Node)
            STATIC_ROOT_NODE->left_son_ptr = *root;
    }
    return;
}

/**
 * @brief 递归逻辑删除盒内有效点并维护重建条件。
 * @param root 子树节点或根指针槽；双指针用于允许替换/置空子树。
 * @param boxpoint 操作/查询盒，各轴为 [min,max)。
 * @param allow_rebuild 是否允许本次递归触发重建；日志回放使用 false。
 * @param is_downsample 是否将失效标记为下采样删除，禁止普通恢复。
 * @return 新删除的有效点数。
 * @note 盒完全覆盖子树时使用懒标记；下采样标记不可被普通盒恢复。部分提前返回路径保留 working_flag=true。
 */
template <typename PointType>
int KD_TREE<PointType>::Delete_by_range(KD_TREE_NODE **root, BoxPointType boxpoint, bool allow_rebuild, bool is_downsample)
{
    if ((*root) == nullptr || (*root)->tree_deleted)
        return 0;
    (*root)->working_flag = true;
    Push_Down(*root);
    int tmp_counter = 0;
    if (boxpoint.vertex_max[0] <= (*root)->node_range_x[0] || boxpoint.vertex_min[0] > (*root)->node_range_x[1])
        return 0;
    if (boxpoint.vertex_max[1] <= (*root)->node_range_y[0] || boxpoint.vertex_min[1] > (*root)->node_range_y[1])
        return 0;
    if (boxpoint.vertex_max[2] <= (*root)->node_range_z[0] || boxpoint.vertex_min[2] > (*root)->node_range_z[1])
        return 0;
    if (boxpoint.vertex_min[0] <= (*root)->node_range_x[0] && boxpoint.vertex_max[0] > (*root)->node_range_x[1] && boxpoint.vertex_min[1] <= (*root)->node_range_y[0] && boxpoint.vertex_max[1] > (*root)->node_range_y[1] && boxpoint.vertex_min[2] <= (*root)->node_range_z[0] && boxpoint.vertex_max[2] > (*root)->node_range_z[1])
    {
        (*root)->tree_deleted = true;
        (*root)->point_deleted = true;
        (*root)->need_push_down_to_left = true;
        (*root)->need_push_down_to_right = true;
        tmp_counter = (*root)->TreeSize - (*root)->invalid_point_num;
        (*root)->invalid_point_num = (*root)->TreeSize;
        if (is_downsample)
        {
            (*root)->tree_downsample_deleted = true;
            (*root)->point_downsample_deleted = true;
            (*root)->down_del_num = (*root)->TreeSize;
        }
        return tmp_counter;
    }
    if (!(*root)->point_deleted && boxpoint.vertex_min[0] <= (*root)->point.x && boxpoint.vertex_max[0] > (*root)->point.x && boxpoint.vertex_min[1] <= (*root)->point.y && boxpoint.vertex_max[1] > (*root)->point.y && boxpoint.vertex_min[2] <= (*root)->point.z && boxpoint.vertex_max[2] > (*root)->point.z)
    {
        (*root)->point_deleted = true;
        tmp_counter += 1;
        if (is_downsample)
            (*root)->point_downsample_deleted = true;
    }
    Operation_Logger_Type delete_box_log;
    struct timespec Timeout;
    if (is_downsample)
        delete_box_log.op = DOWNSAMPLE_DELETE;
    else
        delete_box_log.op = DELETE_BOX;
    delete_box_log.boxpoint = boxpoint;
    if ((Rebuild_Ptr == nullptr) || (*root)->left_son_ptr != *Rebuild_Ptr)
    {
        tmp_counter += Delete_by_range(&((*root)->left_son_ptr), boxpoint, allow_rebuild, is_downsample);
    }
    else
    {
        pthread_mutex_lock(&working_flag_mutex);
        tmp_counter += Delete_by_range(&((*root)->left_son_ptr), boxpoint, false, is_downsample);
        if (rebuild_flag)
        {
            pthread_mutex_lock(&rebuild_logger_mutex_lock);
            Rebuild_Logger.push(delete_box_log);
            pthread_mutex_unlock(&rebuild_logger_mutex_lock);
        }
        pthread_mutex_unlock(&working_flag_mutex);
    }
    if ((Rebuild_Ptr == nullptr) || (*root)->right_son_ptr != *Rebuild_Ptr)
    {
        tmp_counter += Delete_by_range(&((*root)->right_son_ptr), boxpoint, allow_rebuild, is_downsample);
    }
    else
    {
        pthread_mutex_lock(&working_flag_mutex);
        tmp_counter += Delete_by_range(&((*root)->right_son_ptr), boxpoint, false, is_downsample);
        if (rebuild_flag)
        {
            pthread_mutex_lock(&rebuild_logger_mutex_lock);
            Rebuild_Logger.push(delete_box_log);
            pthread_mutex_unlock(&rebuild_logger_mutex_lock);
        }
        pthread_mutex_unlock(&working_flag_mutex);
    }
    Update(*root);
    if (Rebuild_Ptr != nullptr && *Rebuild_Ptr == *root && (*root)->TreeSize < Multi_Thread_Rebuild_Point_Num)
        Rebuild_Ptr = nullptr;
    bool need_rebuild = allow_rebuild & Criterion_Check((*root));
    if (need_rebuild)
        Rebuild(root);
    if ((*root) != nullptr)
        (*root)->working_flag = false;
    return tmp_counter;
}

/**
 * @brief 沿分割轴查找并逻辑删除匹配点。
 * @param root 子树节点或根指针槽；双指针用于允许替换/置空子树。
 * @param point 查询、插入或删除的点；空间比较使用 x/y/z，插入时保存完整点值。
 * @param allow_rebuild 是否允许本次递归触发重建；日志回放使用 false。
 * @note 命中后提前返回，不立即释放节点；部分提前返回路径未复位 working_flag。
 */
template <typename PointType>
void KD_TREE<PointType>::Delete_by_point(KD_TREE_NODE **root, PointType point, bool allow_rebuild)
{
    if ((*root) == nullptr || (*root)->tree_deleted)
        return;
    (*root)->working_flag = true;
    Push_Down(*root);
    if (same_point((*root)->point, point) && !(*root)->point_deleted)
    {
        (*root)->point_deleted = true;
        (*root)->invalid_point_num += 1;
        if ((*root)->invalid_point_num == (*root)->TreeSize)
            (*root)->tree_deleted = true;
        return;
    }
    Operation_Logger_Type delete_log;
    struct timespec Timeout;
    delete_log.op = DELETE_POINT;
    delete_log.point = point;
    if (((*root)->division_axis == 0 && point.x < (*root)->point.x) || ((*root)->division_axis == 1 && point.y < (*root)->point.y) || ((*root)->division_axis == 2 && point.z < (*root)->point.z))
    {
        if ((Rebuild_Ptr == nullptr) || (*root)->left_son_ptr != *Rebuild_Ptr)
        {
            Delete_by_point(&(*root)->left_son_ptr, point, allow_rebuild);
        }
        else
        {
            pthread_mutex_lock(&working_flag_mutex);
            Delete_by_point(&(*root)->left_son_ptr, point, false);
            if (rebuild_flag)
            {
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(delete_log);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
            pthread_mutex_unlock(&working_flag_mutex);
        }
    }
    else
    {
        if ((Rebuild_Ptr == nullptr) || (*root)->right_son_ptr != *Rebuild_Ptr)
        {
            Delete_by_point(&(*root)->right_son_ptr, point, allow_rebuild);
        }
        else
        {
            pthread_mutex_lock(&working_flag_mutex);
            Delete_by_point(&(*root)->right_son_ptr, point, false);
            if (rebuild_flag)
            {
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(delete_log);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
            pthread_mutex_unlock(&working_flag_mutex);
        }
    }
    Update(*root);
    if (Rebuild_Ptr != nullptr && *Rebuild_Ptr == *root && (*root)->TreeSize < Multi_Thread_Rebuild_Point_Num)
        Rebuild_Ptr = nullptr;
    bool need_rebuild = allow_rebuild & Criterion_Check((*root));
    if (need_rebuild)
        Rebuild(root);
    if ((*root) != nullptr)
        (*root)->working_flag = false;
    return;
}

/**
 * @brief 递归撤销盒内普通删除标记。
 * @param root 子树节点或根指针槽；双指针用于允许替换/置空子树。
 * @param boxpoint 操作/查询盒，各轴为 [min,max)。
 * @param allow_rebuild 是否允许本次递归触发重建；日志回放使用 false。
 * @note 下采样删除状态保留；部分提前返回路径未复位 working_flag。
 */
template <typename PointType>
void KD_TREE<PointType>::Add_by_range(KD_TREE_NODE **root, BoxPointType boxpoint, bool allow_rebuild)
{
    if ((*root) == nullptr)
        return;
    (*root)->working_flag = true;
    Push_Down(*root);
    if (boxpoint.vertex_max[0] <= (*root)->node_range_x[0] || boxpoint.vertex_min[0] > (*root)->node_range_x[1])
        return;
    if (boxpoint.vertex_max[1] <= (*root)->node_range_y[0] || boxpoint.vertex_min[1] > (*root)->node_range_y[1])
        return;
    if (boxpoint.vertex_max[2] <= (*root)->node_range_z[0] || boxpoint.vertex_min[2] > (*root)->node_range_z[1])
        return;
    if (boxpoint.vertex_min[0] <= (*root)->node_range_x[0] && boxpoint.vertex_max[0] > (*root)->node_range_x[1] && boxpoint.vertex_min[1] <= (*root)->node_range_y[0] && boxpoint.vertex_max[1] > (*root)->node_range_y[1] && boxpoint.vertex_min[2] <= (*root)->node_range_z[0] && boxpoint.vertex_max[2] > (*root)->node_range_z[1])
    {
        (*root)->tree_deleted = false || (*root)->tree_downsample_deleted;
        (*root)->point_deleted = false || (*root)->point_downsample_deleted;
        (*root)->need_push_down_to_left = true;
        (*root)->need_push_down_to_right = true;
        (*root)->invalid_point_num = (*root)->down_del_num;
        return;
    }
    if (boxpoint.vertex_min[0] <= (*root)->point.x && boxpoint.vertex_max[0] > (*root)->point.x && boxpoint.vertex_min[1] <= (*root)->point.y && boxpoint.vertex_max[1] > (*root)->point.y && boxpoint.vertex_min[2] <= (*root)->point.z && boxpoint.vertex_max[2] > (*root)->point.z)
    {
        (*root)->point_deleted = (*root)->point_downsample_deleted;
    }
    Operation_Logger_Type add_box_log;
    struct timespec Timeout;
    add_box_log.op = ADD_BOX;
    add_box_log.boxpoint = boxpoint;
    if ((Rebuild_Ptr == nullptr) || (*root)->left_son_ptr != *Rebuild_Ptr)
    {
        Add_by_range(&((*root)->left_son_ptr), boxpoint, allow_rebuild);
    }
    else
    {
        pthread_mutex_lock(&working_flag_mutex);
        Add_by_range(&((*root)->left_son_ptr), boxpoint, false);
        if (rebuild_flag)
        {
            pthread_mutex_lock(&rebuild_logger_mutex_lock);
            Rebuild_Logger.push(add_box_log);
            pthread_mutex_unlock(&rebuild_logger_mutex_lock);
        }
        pthread_mutex_unlock(&working_flag_mutex);
    }
    if ((Rebuild_Ptr == nullptr) || (*root)->right_son_ptr != *Rebuild_Ptr)
    {
        Add_by_range(&((*root)->right_son_ptr), boxpoint, allow_rebuild);
    }
    else
    {
        pthread_mutex_lock(&working_flag_mutex);
        Add_by_range(&((*root)->right_son_ptr), boxpoint, false);
        if (rebuild_flag)
        {
            pthread_mutex_lock(&rebuild_logger_mutex_lock);
            Rebuild_Logger.push(add_box_log);
            pthread_mutex_unlock(&rebuild_logger_mutex_lock);
        }
        pthread_mutex_unlock(&working_flag_mutex);
    }
    Update(*root);
    if (Rebuild_Ptr != nullptr && *Rebuild_Ptr == *root && (*root)->TreeSize < Multi_Thread_Rebuild_Point_Num)
        Rebuild_Ptr = nullptr;
    bool need_rebuild = allow_rebuild & Criterion_Check((*root));
    if (need_rebuild)
        Rebuild(root);
    if ((*root) != nullptr)
        (*root)->working_flag = false;
    return;
}

/**
 * @brief 沿分割轴插入新节点并回溯维护统计。
 * @param root 子树节点或根指针槽；双指针用于允许替换/置空子树。
 * @param point 查询、插入或删除的点；空间比较使用 x/y/z，插入时保存完整点值。
 * @param allow_rebuild 是否允许本次递归触发重建；日志回放使用 false。
 * @param father_axis 父节点分割轴，新节点按下一轴循环分割。
 * @note 新节点的分割轴为 (father_axis+1)%3，重建后才重新按最长轴选择。
 */
template <typename PointType>
void KD_TREE<PointType>::Add_by_point(KD_TREE_NODE **root, PointType point, bool allow_rebuild, int father_axis)
{
    if (*root == nullptr)
    {
        *root = new KD_TREE_NODE;
        InitTreeNode(*root);
        (*root)->point = point;
        (*root)->division_axis = (father_axis + 1) % 3;
        Update(*root);
        return;
    }
    (*root)->working_flag = true;
    Operation_Logger_Type add_log;
    struct timespec Timeout;
    add_log.op = ADD_POINT;
    add_log.point = point;
    Push_Down(*root);
    if (((*root)->division_axis == 0 && point.x < (*root)->point.x) || ((*root)->division_axis == 1 && point.y < (*root)->point.y) || ((*root)->division_axis == 2 && point.z < (*root)->point.z))
    {
        if ((Rebuild_Ptr == nullptr) || (*root)->left_son_ptr != *Rebuild_Ptr)
        {
            Add_by_point(&(*root)->left_son_ptr, point, allow_rebuild, (*root)->division_axis);
        }
        else
        {
            pthread_mutex_lock(&working_flag_mutex);
            Add_by_point(&(*root)->left_son_ptr, point, false, (*root)->division_axis);
            if (rebuild_flag)
            {
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(add_log);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
            pthread_mutex_unlock(&working_flag_mutex);
        }
    }
    else
    {
        if ((Rebuild_Ptr == nullptr) || (*root)->right_son_ptr != *Rebuild_Ptr)
        {
            Add_by_point(&(*root)->right_son_ptr, point, allow_rebuild, (*root)->division_axis);
        }
        else
        {
            pthread_mutex_lock(&working_flag_mutex);
            Add_by_point(&(*root)->right_son_ptr, point, false, (*root)->division_axis);
            if (rebuild_flag)
            {
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(add_log);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
            pthread_mutex_unlock(&working_flag_mutex);
        }
    }
    Update(*root);
    if (Rebuild_Ptr != nullptr && *Rebuild_Ptr == *root && (*root)->TreeSize < Multi_Thread_Rebuild_Point_Num)
        Rebuild_Ptr = nullptr;
    bool need_rebuild = allow_rebuild & Criterion_Check((*root));
    if (need_rebuild)
        Rebuild(root);
    if ((*root) != nullptr)
        (*root)->working_flag = false;
    return;
}

/**
 * @brief 通过包围盒距离下界剪枝执行 k 最近邻搜索。
 * @param root 子树节点或根指针槽；双指针用于允许替换/置空子树。
 * @param k_nearest 最大近邻数量，须>0。
 * @param point 查询、插入或删除的点；空间比较使用 x/y/z，插入时保存完整点值。
 * @param q 共享本次查询候选的最大堆。
 * @param max_dist 最大允许距离，使用长度单位，内部比较其平方。
 * @note q 是最大堆，堆顶为最差候选；必须先传播懒标记，以免返回已删除点。
 */
template <typename PointType>
void KD_TREE<PointType>::Search(KD_TREE_NODE *root, int k_nearest, PointType point, MANUAL_HEAP &q, float max_dist)
{
    if (root == nullptr || root->tree_deleted)
        return;
    /* 点到包围盒的距离平方是子树所有点距离的下界；超过半径即可剪枝，候选满 k 后还可用堆顶最差距离剪枝。 */
    float cur_dist = calc_box_dist(root, point);
    float max_dist_sqr = max_dist * max_dist;
    if (cur_dist > max_dist_sqr)
        return;
    int retval;
    if (root->need_push_down_to_left || root->need_push_down_to_right)
    {
        retval = pthread_mutex_trylock(&(root->push_down_mutex_lock));
        if (retval == 0)
        {
            Push_Down(root);
            pthread_mutex_unlock(&(root->push_down_mutex_lock));
        }
        else
        {
            pthread_mutex_lock(&(root->push_down_mutex_lock));
            pthread_mutex_unlock(&(root->push_down_mutex_lock));
        }
    }
    if (!root->point_deleted)
    {
        float dist = calc_dist(point, root->point);
        if (dist <= max_dist_sqr && (q.size() < k_nearest || dist < q.top().dist))
        {
            if (q.size() >= k_nearest)
                q.pop();
            PointType_CMP current_point{root->point, dist};
            q.push(current_point);
        }
    }
    int cur_search_counter;
    float dist_left_node = calc_box_dist(root->left_son_ptr, point);
    float dist_right_node = calc_box_dist(root->right_son_ptr, point);
    if (q.size() < k_nearest || dist_left_node < q.top().dist && dist_right_node < q.top().dist)
    {
        if (dist_left_node <= dist_right_node)
        {
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->left_son_ptr)
            {
                Search(root->left_son_ptr, k_nearest, point, q, max_dist);
            }
            else
            {
                pthread_mutex_lock(&search_flag_mutex);
                while (search_mutex_counter == -1)
                {
                    pthread_mutex_unlock(&search_flag_mutex);
                    usleep(1);
                    pthread_mutex_lock(&search_flag_mutex);
                }
                search_mutex_counter += 1;
                pthread_mutex_unlock(&search_flag_mutex);
                Search(root->left_son_ptr, k_nearest, point, q, max_dist);
                pthread_mutex_lock(&search_flag_mutex);
                search_mutex_counter -= 1;
                pthread_mutex_unlock(&search_flag_mutex);
            }
            if (q.size() < k_nearest || dist_right_node < q.top().dist)
            {
                if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->right_son_ptr)
                {
                    Search(root->right_son_ptr, k_nearest, point, q, max_dist);
                }
                else
                {
                    pthread_mutex_lock(&search_flag_mutex);
                    while (search_mutex_counter == -1)
                    {
                        pthread_mutex_unlock(&search_flag_mutex);
                        usleep(1);
                        pthread_mutex_lock(&search_flag_mutex);
                    }
                    search_mutex_counter += 1;
                    pthread_mutex_unlock(&search_flag_mutex);
                    Search(root->right_son_ptr, k_nearest, point, q, max_dist);
                    pthread_mutex_lock(&search_flag_mutex);
                    search_mutex_counter -= 1;
                    pthread_mutex_unlock(&search_flag_mutex);
                }
            }
        }
        else
        {
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->right_son_ptr)
            {
                Search(root->right_son_ptr, k_nearest, point, q, max_dist);
            }
            else
            {
                pthread_mutex_lock(&search_flag_mutex);
                while (search_mutex_counter == -1)
                {
                    pthread_mutex_unlock(&search_flag_mutex);
                    usleep(1);
                    pthread_mutex_lock(&search_flag_mutex);
                }
                search_mutex_counter += 1;
                pthread_mutex_unlock(&search_flag_mutex);
                Search(root->right_son_ptr, k_nearest, point, q, max_dist);
                pthread_mutex_lock(&search_flag_mutex);
                search_mutex_counter -= 1;
                pthread_mutex_unlock(&search_flag_mutex);
            }
            if (q.size() < k_nearest || dist_left_node < q.top().dist)
            {
                if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->left_son_ptr)
                {
                    Search(root->left_son_ptr, k_nearest, point, q, max_dist);
                }
                else
                {
                    pthread_mutex_lock(&search_flag_mutex);
                    while (search_mutex_counter == -1)
                    {
                        pthread_mutex_unlock(&search_flag_mutex);
                        usleep(1);
                        pthread_mutex_lock(&search_flag_mutex);
                    }
                    search_mutex_counter += 1;
                    pthread_mutex_unlock(&search_flag_mutex);
                    Search(root->left_son_ptr, k_nearest, point, q, max_dist);
                    pthread_mutex_lock(&search_flag_mutex);
                    search_mutex_counter -= 1;
                    pthread_mutex_unlock(&search_flag_mutex);
                }
            }
        }
    }
    else
    {
        if (dist_left_node < q.top().dist)
        {
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->left_son_ptr)
            {
                Search(root->left_son_ptr, k_nearest, point, q, max_dist);
            }
            else
            {
                pthread_mutex_lock(&search_flag_mutex);
                while (search_mutex_counter == -1)
                {
                    pthread_mutex_unlock(&search_flag_mutex);
                    usleep(1);
                    pthread_mutex_lock(&search_flag_mutex);
                }
                search_mutex_counter += 1;
                pthread_mutex_unlock(&search_flag_mutex);
                Search(root->left_son_ptr, k_nearest, point, q, max_dist);
                pthread_mutex_lock(&search_flag_mutex);
                search_mutex_counter -= 1;
                pthread_mutex_unlock(&search_flag_mutex);
            }
        }
        if (dist_right_node < q.top().dist)
        {
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->right_son_ptr)
            {
                Search(root->right_son_ptr, k_nearest, point, q, max_dist);
            }
            else
            {
                pthread_mutex_lock(&search_flag_mutex);
                while (search_mutex_counter == -1)
                {
                    pthread_mutex_unlock(&search_flag_mutex);
                    usleep(1);
                    pthread_mutex_lock(&search_flag_mutex);
                }
                search_mutex_counter += 1;
                pthread_mutex_unlock(&search_flag_mutex);
                Search(root->right_son_ptr, k_nearest, point, q, max_dist);
                pthread_mutex_lock(&search_flag_mutex);
                search_mutex_counter -= 1;
                pthread_mutex_unlock(&search_flag_mutex);
            }
        }
    }
    return;
}

/**
 * @brief 递归收集左闭右开盒内有效点。
 * @param root 子树节点或根指针槽；双指针用于允许替换/置空子树。
 * @param boxpoint 操作/查询盒，各轴为 [min,max)。
 * @param Storage 点数组；公开查询清空，内部遍历/flatten 追加，BuildTree 可重排。
 * @note 向 Storage 追加；完整覆盖子树时通过 flatten 收集。
 */
template <typename PointType>
void KD_TREE<PointType>::Search_by_range(KD_TREE_NODE *root, BoxPointType boxpoint, PointVector &Storage)
{
    if (root == nullptr)
        return;
    Push_Down(root);
    if (boxpoint.vertex_max[0] <= root->node_range_x[0] || boxpoint.vertex_min[0] > root->node_range_x[1])
        return;
    if (boxpoint.vertex_max[1] <= root->node_range_y[0] || boxpoint.vertex_min[1] > root->node_range_y[1])
        return;
    if (boxpoint.vertex_max[2] <= root->node_range_z[0] || boxpoint.vertex_min[2] > root->node_range_z[1])
        return;
    if (boxpoint.vertex_min[0] <= root->node_range_x[0] && boxpoint.vertex_max[0] > root->node_range_x[1] && boxpoint.vertex_min[1] <= root->node_range_y[0] && boxpoint.vertex_max[1] > root->node_range_y[1] && boxpoint.vertex_min[2] <= root->node_range_z[0] && boxpoint.vertex_max[2] > root->node_range_z[1])
    {
        flatten(root, Storage, NOT_RECORD);
        return;
    }
    if (boxpoint.vertex_min[0] <= root->point.x && boxpoint.vertex_max[0] > root->point.x && boxpoint.vertex_min[1] <= root->point.y && boxpoint.vertex_max[1] > root->point.y && boxpoint.vertex_min[2] <= root->point.z && boxpoint.vertex_max[2] > root->point.z)
    {
        if (!root->point_deleted)
            Storage.push_back(root->point);
    }
    if ((Rebuild_Ptr == nullptr) || root->left_son_ptr != *Rebuild_Ptr)
    {
        Search_by_range(root->left_son_ptr, boxpoint, Storage);
    }
    else
    {
        pthread_mutex_lock(&search_flag_mutex);
        Search_by_range(root->left_son_ptr, boxpoint, Storage);
        pthread_mutex_unlock(&search_flag_mutex);
    }
    if ((Rebuild_Ptr == nullptr) || root->right_son_ptr != *Rebuild_Ptr)
    {
        Search_by_range(root->right_son_ptr, boxpoint, Storage);
    }
    else
    {
        pthread_mutex_lock(&search_flag_mutex);
        Search_by_range(root->right_son_ptr, boxpoint, Storage);
        pthread_mutex_unlock(&search_flag_mutex);
    }
    return;
}

/**
 * @brief 按子树包围球剪枝收集闭球内有效点。
 * @param root 子树节点或根指针槽；双指针用于允许替换/置空子树。
 * @param point 查询、插入或删除的点；空间比较使用 x/y/z，插入时保存完整点值。
 * @param radius 闭球搜索半径，须非负。
 * @param Storage 点数组；公开查询清空，内部遍历/flatten 追加，BuildTree 可重排。
 * @note 向 Storage 追加；radius 与点坐标同单位。
 */
template <typename PointType>
void KD_TREE<PointType>::Search_by_radius(KD_TREE_NODE *root, PointType point, float radius, PointVector &Storage)
{
    if (root == nullptr)
        return;
    Push_Down(root);
    PointType range_center;
    range_center.x = (root->node_range_x[0] + root->node_range_x[1]) * 0.5;
    range_center.y = (root->node_range_y[0] + root->node_range_y[1]) * 0.5;
    range_center.z = (root->node_range_z[0] + root->node_range_z[1]) * 0.5;
    float dist = sqrt(calc_dist(range_center, point));
    if (dist > radius + sqrt(root->radius_sq)) return;
    if (dist <= radius - sqrt(root->radius_sq)) 
    {
        flatten(root, Storage, NOT_RECORD);
        return;
    }
    if (!root->point_deleted && calc_dist(root->point, point) <= radius * radius){
        Storage.push_back(root->point);
    }
    if ((Rebuild_Ptr == nullptr) || root->left_son_ptr != *Rebuild_Ptr)
    {
        Search_by_radius(root->left_son_ptr, point, radius, Storage);
    }
    else
    {
        pthread_mutex_lock(&search_flag_mutex);
        Search_by_radius(root->left_son_ptr, point, radius, Storage);
        pthread_mutex_unlock(&search_flag_mutex);
    }
    if ((Rebuild_Ptr == nullptr) || root->right_son_ptr != *Rebuild_Ptr)
    {
        Search_by_radius(root->right_son_ptr, point, radius, Storage);
    }
    else
    {
        pthread_mutex_lock(&search_flag_mutex);
        Search_by_radius(root->right_son_ptr, point, radius, Storage);
        pthread_mutex_unlock(&search_flag_mutex);
    }    
    return;
}

/**
 * @brief 按删除比例或子树失衡比例判断是否需要重建。
 * @param root 子树节点或根指针槽；双指针用于允许替换/置空子树。
 * @return 节点数>10 且删除比例超阈值或左右规模比例超界时为 true。
 * @note 要求有效非空节点，阈值比较使用严格大于/小于。
 */
template <typename PointType>
bool KD_TREE<PointType>::Criterion_Check(KD_TREE_NODE *root)
{
    if (root->TreeSize <= Minimal_Unbalanced_Tree_Size)
    {
        return false;
    }
    float balance_evaluation = 0.0f;
    float delete_evaluation = 0.0f;
    KD_TREE_NODE *son_ptr = root->left_son_ptr;
    if (son_ptr == nullptr)
        son_ptr = root->right_son_ptr;
    delete_evaluation = float(root->invalid_point_num) / root->TreeSize;
    balance_evaluation = float(son_ptr->TreeSize) / (root->TreeSize - 1);
    if (delete_evaluation > delete_criterion_param)
    {
        return true;
    }
    if (balance_evaluation > balance_criterion_param || balance_evaluation < 1 - balance_criterion_param)
    {
        return true;
    }
    return false;
}

/**
 * @brief 将整棵子树的懒删除/恢复标记传播到子节点。
 * @param root 子树节点或根指针槽；双指针用于允许替换/置空子树。
 * @note 下采样删除通过 OR 保留；重建目标子树的传播还需记入日志，不能仅修改旧树。
 */
template <typename PointType>
void KD_TREE<PointType>::Push_Down(KD_TREE_NODE *root)
{
    if (root == nullptr)
        return;
    /* 先传播整树懒标记再访问子节点；下采样标记不可被普通 ADD_BOX 撤销，重建中的传播必须同时写入操作日志。 */
    Operation_Logger_Type operation;
    operation.op = PUSH_DOWN;
    operation.tree_deleted = root->tree_deleted;
    operation.tree_downsample_deleted = root->tree_downsample_deleted;
    if (root->need_push_down_to_left && root->left_son_ptr != nullptr)
    {
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->left_son_ptr)
        {
            root->left_son_ptr->tree_downsample_deleted |= root->tree_downsample_deleted;
            root->left_son_ptr->point_downsample_deleted |= root->tree_downsample_deleted;
            root->left_son_ptr->tree_deleted = root->tree_deleted || root->left_son_ptr->tree_downsample_deleted;
            root->left_son_ptr->point_deleted = root->left_son_ptr->tree_deleted || root->left_son_ptr->point_downsample_deleted;
            if (root->tree_downsample_deleted)
                root->left_son_ptr->down_del_num = root->left_son_ptr->TreeSize;
            if (root->tree_deleted)
                root->left_son_ptr->invalid_point_num = root->left_son_ptr->TreeSize;
            else
                root->left_son_ptr->invalid_point_num = root->left_son_ptr->down_del_num;
            root->left_son_ptr->need_push_down_to_left = true;
            root->left_son_ptr->need_push_down_to_right = true;
            root->need_push_down_to_left = false;
        }
        else
        {
            pthread_mutex_lock(&working_flag_mutex);
            root->left_son_ptr->tree_downsample_deleted |= root->tree_downsample_deleted;
            root->left_son_ptr->point_downsample_deleted |= root->tree_downsample_deleted;
            root->left_son_ptr->tree_deleted = root->tree_deleted || root->left_son_ptr->tree_downsample_deleted;
            root->left_son_ptr->point_deleted = root->left_son_ptr->tree_deleted || root->left_son_ptr->point_downsample_deleted;
            if (root->tree_downsample_deleted)
                root->left_son_ptr->down_del_num = root->left_son_ptr->TreeSize;
            if (root->tree_deleted)
                root->left_son_ptr->invalid_point_num = root->left_son_ptr->TreeSize;
            else
                root->left_son_ptr->invalid_point_num = root->left_son_ptr->down_del_num;
            root->left_son_ptr->need_push_down_to_left = true;
            root->left_son_ptr->need_push_down_to_right = true;
            if (rebuild_flag)
            {
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(operation);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
            root->need_push_down_to_left = false;
            pthread_mutex_unlock(&working_flag_mutex);
        }
    }
    if (root->need_push_down_to_right && root->right_son_ptr != nullptr)
    {
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->right_son_ptr)
        {
            root->right_son_ptr->tree_downsample_deleted |= root->tree_downsample_deleted;
            root->right_son_ptr->point_downsample_deleted |= root->tree_downsample_deleted;
            root->right_son_ptr->tree_deleted = root->tree_deleted || root->right_son_ptr->tree_downsample_deleted;
            root->right_son_ptr->point_deleted = root->right_son_ptr->tree_deleted || root->right_son_ptr->point_downsample_deleted;
            if (root->tree_downsample_deleted)
                root->right_son_ptr->down_del_num = root->right_son_ptr->TreeSize;
            if (root->tree_deleted)
                root->right_son_ptr->invalid_point_num = root->right_son_ptr->TreeSize;
            else
                root->right_son_ptr->invalid_point_num = root->right_son_ptr->down_del_num;
            root->right_son_ptr->need_push_down_to_left = true;
            root->right_son_ptr->need_push_down_to_right = true;
            root->need_push_down_to_right = false;
        }
        else
        {
            pthread_mutex_lock(&working_flag_mutex);
            root->right_son_ptr->tree_downsample_deleted |= root->tree_downsample_deleted;
            root->right_son_ptr->point_downsample_deleted |= root->tree_downsample_deleted;
            root->right_son_ptr->tree_deleted = root->tree_deleted || root->right_son_ptr->tree_downsample_deleted;
            root->right_son_ptr->point_deleted = root->right_son_ptr->tree_deleted || root->right_son_ptr->point_downsample_deleted;
            if (root->tree_downsample_deleted)
                root->right_son_ptr->down_del_num = root->right_son_ptr->TreeSize;
            if (root->tree_deleted)
                root->right_son_ptr->invalid_point_num = root->right_son_ptr->TreeSize;
            else
                root->right_son_ptr->invalid_point_num = root->right_son_ptr->down_del_num;
            root->right_son_ptr->need_push_down_to_left = true;
            root->right_son_ptr->need_push_down_to_right = true;
            if (rebuild_flag)
            {
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(operation);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
            root->need_push_down_to_right = false;
            pthread_mutex_unlock(&working_flag_mutex);
        }
    }
    return;
}

/**
 * @brief 从子节点回算规模、失效计数、边界和父指针。
 * @param root 子树节点或根指针槽；双指针用于允许替换/置空子树。
 * @note 要求非空节点；包围盒按有效点合并，全删时保留几何范围供后续恢复；radius_sq 为盒半对角线长度平方。
 */
template <typename PointType>
void KD_TREE<PointType>::Update(KD_TREE_NODE *root)
{
    KD_TREE_NODE *left_son_ptr = root->left_son_ptr;
    KD_TREE_NODE *right_son_ptr = root->right_son_ptr;
    float tmp_range_x[2] = {INFINITY, -INFINITY};
    float tmp_range_y[2] = {INFINITY, -INFINITY};
    float tmp_range_z[2] = {INFINITY, -INFINITY};
    // Update Tree Size
    if (left_son_ptr != nullptr && right_son_ptr != nullptr)
    {
        root->TreeSize = left_son_ptr->TreeSize + right_son_ptr->TreeSize + 1;
        root->invalid_point_num = left_son_ptr->invalid_point_num + right_son_ptr->invalid_point_num + (root->point_deleted ? 1 : 0);
        root->down_del_num = left_son_ptr->down_del_num + right_son_ptr->down_del_num + (root->point_downsample_deleted ? 1 : 0);
        root->tree_downsample_deleted = left_son_ptr->tree_downsample_deleted & right_son_ptr->tree_downsample_deleted & root->point_downsample_deleted;
        root->tree_deleted = left_son_ptr->tree_deleted && right_son_ptr->tree_deleted && root->point_deleted;
        if (root->tree_deleted || (!left_son_ptr->tree_deleted && !right_son_ptr->tree_deleted && !root->point_deleted))
        {
            tmp_range_x[0] = min(min(left_son_ptr->node_range_x[0], right_son_ptr->node_range_x[0]), root->point.x);
            tmp_range_x[1] = max(max(left_son_ptr->node_range_x[1], right_son_ptr->node_range_x[1]), root->point.x);
            tmp_range_y[0] = min(min(left_son_ptr->node_range_y[0], right_son_ptr->node_range_y[0]), root->point.y);
            tmp_range_y[1] = max(max(left_son_ptr->node_range_y[1], right_son_ptr->node_range_y[1]), root->point.y);
            tmp_range_z[0] = min(min(left_son_ptr->node_range_z[0], right_son_ptr->node_range_z[0]), root->point.z);
            tmp_range_z[1] = max(max(left_son_ptr->node_range_z[1], right_son_ptr->node_range_z[1]), root->point.z);
        }
        else
        {
            if (!left_son_ptr->tree_deleted)
            {
                tmp_range_x[0] = min(tmp_range_x[0], left_son_ptr->node_range_x[0]);
                tmp_range_x[1] = max(tmp_range_x[1], left_son_ptr->node_range_x[1]);
                tmp_range_y[0] = min(tmp_range_y[0], left_son_ptr->node_range_y[0]);
                tmp_range_y[1] = max(tmp_range_y[1], left_son_ptr->node_range_y[1]);
                tmp_range_z[0] = min(tmp_range_z[0], left_son_ptr->node_range_z[0]);
                tmp_range_z[1] = max(tmp_range_z[1], left_son_ptr->node_range_z[1]);
            }
            if (!right_son_ptr->tree_deleted)
            {
                tmp_range_x[0] = min(tmp_range_x[0], right_son_ptr->node_range_x[0]);
                tmp_range_x[1] = max(tmp_range_x[1], right_son_ptr->node_range_x[1]);
                tmp_range_y[0] = min(tmp_range_y[0], right_son_ptr->node_range_y[0]);
                tmp_range_y[1] = max(tmp_range_y[1], right_son_ptr->node_range_y[1]);
                tmp_range_z[0] = min(tmp_range_z[0], right_son_ptr->node_range_z[0]);
                tmp_range_z[1] = max(tmp_range_z[1], right_son_ptr->node_range_z[1]);
            }
            if (!root->point_deleted)
            {
                tmp_range_x[0] = min(tmp_range_x[0], root->point.x);
                tmp_range_x[1] = max(tmp_range_x[1], root->point.x);
                tmp_range_y[0] = min(tmp_range_y[0], root->point.y);
                tmp_range_y[1] = max(tmp_range_y[1], root->point.y);
                tmp_range_z[0] = min(tmp_range_z[0], root->point.z);
                tmp_range_z[1] = max(tmp_range_z[1], root->point.z);
            }
        }
    }
    else if (left_son_ptr != nullptr)
    {
        root->TreeSize = left_son_ptr->TreeSize + 1;
        root->invalid_point_num = left_son_ptr->invalid_point_num + (root->point_deleted ? 1 : 0);
        root->down_del_num = left_son_ptr->down_del_num + (root->point_downsample_deleted ? 1 : 0);
        root->tree_downsample_deleted = left_son_ptr->tree_downsample_deleted & root->point_downsample_deleted;
        root->tree_deleted = left_son_ptr->tree_deleted && root->point_deleted;
        if (root->tree_deleted || (!left_son_ptr->tree_deleted && !root->point_deleted))
        {
            tmp_range_x[0] = min(left_son_ptr->node_range_x[0], root->point.x);
            tmp_range_x[1] = max(left_son_ptr->node_range_x[1], root->point.x);
            tmp_range_y[0] = min(left_son_ptr->node_range_y[0], root->point.y);
            tmp_range_y[1] = max(left_son_ptr->node_range_y[1], root->point.y);
            tmp_range_z[0] = min(left_son_ptr->node_range_z[0], root->point.z);
            tmp_range_z[1] = max(left_son_ptr->node_range_z[1], root->point.z);
        }
        else
        {
            if (!left_son_ptr->tree_deleted)
            {
                tmp_range_x[0] = min(tmp_range_x[0], left_son_ptr->node_range_x[0]);
                tmp_range_x[1] = max(tmp_range_x[1], left_son_ptr->node_range_x[1]);
                tmp_range_y[0] = min(tmp_range_y[0], left_son_ptr->node_range_y[0]);
                tmp_range_y[1] = max(tmp_range_y[1], left_son_ptr->node_range_y[1]);
                tmp_range_z[0] = min(tmp_range_z[0], left_son_ptr->node_range_z[0]);
                tmp_range_z[1] = max(tmp_range_z[1], left_son_ptr->node_range_z[1]);
            }
            if (!root->point_deleted)
            {
                tmp_range_x[0] = min(tmp_range_x[0], root->point.x);
                tmp_range_x[1] = max(tmp_range_x[1], root->point.x);
                tmp_range_y[0] = min(tmp_range_y[0], root->point.y);
                tmp_range_y[1] = max(tmp_range_y[1], root->point.y);
                tmp_range_z[0] = min(tmp_range_z[0], root->point.z);
                tmp_range_z[1] = max(tmp_range_z[1], root->point.z);
            }
        }
    }
    else if (right_son_ptr != nullptr)
    {
        root->TreeSize = right_son_ptr->TreeSize + 1;
        root->invalid_point_num = right_son_ptr->invalid_point_num + (root->point_deleted ? 1 : 0);
        root->down_del_num = right_son_ptr->down_del_num + (root->point_downsample_deleted ? 1 : 0);
        root->tree_downsample_deleted = right_son_ptr->tree_downsample_deleted & root->point_downsample_deleted;
        root->tree_deleted = right_son_ptr->tree_deleted && root->point_deleted;
        if (root->tree_deleted || (!right_son_ptr->tree_deleted && !root->point_deleted))
        {
            tmp_range_x[0] = min(right_son_ptr->node_range_x[0], root->point.x);
            tmp_range_x[1] = max(right_son_ptr->node_range_x[1], root->point.x);
            tmp_range_y[0] = min(right_son_ptr->node_range_y[0], root->point.y);
            tmp_range_y[1] = max(right_son_ptr->node_range_y[1], root->point.y);
            tmp_range_z[0] = min(right_son_ptr->node_range_z[0], root->point.z);
            tmp_range_z[1] = max(right_son_ptr->node_range_z[1], root->point.z);
        }
        else
        {
            if (!right_son_ptr->tree_deleted)
            {
                tmp_range_x[0] = min(tmp_range_x[0], right_son_ptr->node_range_x[0]);
                tmp_range_x[1] = max(tmp_range_x[1], right_son_ptr->node_range_x[1]);
                tmp_range_y[0] = min(tmp_range_y[0], right_son_ptr->node_range_y[0]);
                tmp_range_y[1] = max(tmp_range_y[1], right_son_ptr->node_range_y[1]);
                tmp_range_z[0] = min(tmp_range_z[0], right_son_ptr->node_range_z[0]);
                tmp_range_z[1] = max(tmp_range_z[1], right_son_ptr->node_range_z[1]);
            }
            if (!root->point_deleted)
            {
                tmp_range_x[0] = min(tmp_range_x[0], root->point.x);
                tmp_range_x[1] = max(tmp_range_x[1], root->point.x);
                tmp_range_y[0] = min(tmp_range_y[0], root->point.y);
                tmp_range_y[1] = max(tmp_range_y[1], root->point.y);
                tmp_range_z[0] = min(tmp_range_z[0], root->point.z);
                tmp_range_z[1] = max(tmp_range_z[1], root->point.z);
            }
        }
    }
    else
    {
        root->TreeSize = 1;
        root->invalid_point_num = (root->point_deleted ? 1 : 0);
        root->down_del_num = (root->point_downsample_deleted ? 1 : 0);
        root->tree_downsample_deleted = root->point_downsample_deleted;
        root->tree_deleted = root->point_deleted;
        tmp_range_x[0] = root->point.x;
        tmp_range_x[1] = root->point.x;
        tmp_range_y[0] = root->point.y;
        tmp_range_y[1] = root->point.y;
        tmp_range_z[0] = root->point.z;
        tmp_range_z[1] = root->point.z;
    }
    memcpy(root->node_range_x, tmp_range_x, sizeof(tmp_range_x));
    memcpy(root->node_range_y, tmp_range_y, sizeof(tmp_range_y));
    memcpy(root->node_range_z, tmp_range_z, sizeof(tmp_range_z));
    float x_L = (root->node_range_x[1] - root->node_range_x[0]) * 0.5;
    float y_L = (root->node_range_y[1] - root->node_range_y[0]) * 0.5;
    float z_L = (root->node_range_z[1] - root->node_range_z[0]) * 0.5;
    /* 包围盒半对角线平方用作外接球半径平方；半径查询的包围球剪枝和最近邻的盒距离剪枝使用不同边界。 */
    root->radius_sq = x_L*x_L + y_L * y_L + z_L * z_L;
    if (left_son_ptr != nullptr)
        left_son_ptr->father_ptr = root;
    if (right_son_ptr != nullptr)
        right_son_ptr->father_ptr = root;
    if (root == Root_Node && root->TreeSize > 3)
    {
        KD_TREE_NODE *son_ptr = root->left_son_ptr;
        if (son_ptr == nullptr)
            son_ptr = root->right_son_ptr;
        float tmp_bal = float(son_ptr->TreeSize) / (root->TreeSize - 1);
        root->alpha_del = float(root->invalid_point_num) / root->TreeSize;
        root->alpha_bal = (tmp_bal >= 0.5 - EPSS) ? tmp_bal : 1 - tmp_bal;
    }
    return;
}

/**
 * @brief 前序收集有效点，并按模式记录普通删除点。
 * @param root 子树节点或根指针槽；双指针用于允许替换/置空子树。
 * @param Storage 点数组；公开查询清空，内部遍历/flatten 追加，BuildTree 可重排。
 * @param storage_type 删除点缓存记录模式。
 * @note 先下推懒标记，向 Storage 追加而不清空；下采样删除点不进入 removed_points 缓存。
 */
template <typename PointType>
void KD_TREE<PointType>::flatten(KD_TREE_NODE *root, PointVector &Storage, delete_point_storage_set storage_type)
{
    if (root == nullptr)
        return;
    Push_Down(root);
    if (!root->point_deleted)
    {
        Storage.push_back(root->point);
    }
    flatten(root->left_son_ptr, Storage, storage_type);
    flatten(root->right_son_ptr, Storage, storage_type);
    switch (storage_type)
    {
    case NOT_RECORD:
        break;
    case DELETE_POINTS_REC:
        if (root->point_deleted && !root->point_downsample_deleted)
        {
            Points_deleted.push_back(root->point);
        }
        break;
    case MULTI_THREAD_REC:
        if (root->point_deleted && !root->point_downsample_deleted)
        {
            Multithread_Points_deleted.push_back(root->point);
        }
        break;
    default:
        break;
    }
    return;
}

/**
 * @brief 递归释放子树节点并将根槽置空。
 * @param root 子树节点或根指针槽；双指针用于允许替换/置空子树。
 * @note 释放前下推标记并销毁各节点互斥锁；调用方保证无并发读取。
 */
template <typename PointType>
void KD_TREE<PointType>::delete_tree_nodes(KD_TREE_NODE **root)
{
    if (*root == nullptr)
        return;
    Push_Down(*root);
    delete_tree_nodes(&(*root)->left_son_ptr);
    delete_tree_nodes(&(*root)->right_son_ptr);

    pthread_mutex_destroy(&(*root)->push_down_mutex_lock);
    delete *root;
    *root = nullptr;

    return;
}

/**
 * @brief 按逐轴绝对差判断两点是否相同。
 * @param a 比较/距离计算的第一个点。
 * @param b 比较/距离计算的第二个点。
 * @return x/y/z 各差值均严格小于 EPSS=1e-6 时为 true。
 * @note 不比较强度、法向量等其他字段。
 */
template <typename PointType>
bool KD_TREE<PointType>::same_point(PointType a, PointType b)
{
    return (fabs(a.x - b.x) < EPSS && fabs(a.y - b.y) < EPSS && fabs(a.z - b.z) < EPSS);
}

/**
 * @brief 计算两点三维欧氏距离平方。
 * @param a 比较/距离计算的第一个点。
 * @param b 比较/距离计算的第二个点。
 * @return dx²+dy²+dz²，单位为点坐标单位的平方。
 */
template <typename PointType>
float KD_TREE<PointType>::calc_dist(PointType a, PointType b)
{
    float dist = 0.0f;
    dist = (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z);
    return dist;
}

/**
 * @brief 计算查询点到子树包围盒的距离平方下界。
 * @param node 待计算距离下界的子树节点，可为空。
 * @param point 查询、插入或删除的点；空间比较使用 x/y/z，插入时保存完整点值。
 * @return 盒内为 0，盒外按各轴越界量平方求和；node=nullptr 时为正无穷。
 */
template <typename PointType>
float KD_TREE<PointType>::calc_box_dist(KD_TREE_NODE *node, PointType point)
{
    if (node == nullptr)
        return INFINITY;
    float min_dist = 0.0;
    if (point.x < node->node_range_x[0])
        min_dist += (point.x - node->node_range_x[0]) * (point.x - node->node_range_x[0]);
    if (point.x > node->node_range_x[1])
        min_dist += (point.x - node->node_range_x[1]) * (point.x - node->node_range_x[1]);
    if (point.y < node->node_range_y[0])
        min_dist += (point.y - node->node_range_y[0]) * (point.y - node->node_range_y[0]);
    if (point.y > node->node_range_y[1])
        min_dist += (point.y - node->node_range_y[1]) * (point.y - node->node_range_y[1]);
    if (point.z < node->node_range_z[0])
        min_dist += (point.z - node->node_range_z[0]) * (point.z - node->node_range_z[0]);
    if (point.z > node->node_range_z[1])
        min_dist += (point.z - node->node_range_z[1]) * (point.z - node->node_range_z[1]);
    return min_dist;
}
/**
 * @brief 按 x 坐标严格排序两点。
 * @param a 比较/距离计算的第一个点。
 * @param b 比较/距离计算的第二个点。
 * @return a.x<b.x。
 */
template <typename PointType>
bool KD_TREE<PointType>::point_cmp_x(PointType a, PointType b) { return a.x < b.x; }
/**
 * @brief 按 y 坐标严格排序两点。
 * @param a 比较/距离计算的第一个点。
 * @param b 比较/距离计算的第二个点。
 * @return a.y<b.y。
 */
template <typename PointType>
bool KD_TREE<PointType>::point_cmp_y(PointType a, PointType b) { return a.y < b.y; }
/**
 * @brief 按 z 坐标严格排序两点。
 * @param a 比较/距离计算的第一个点。
 * @param b 比较/距离计算的第二个点。
 * @return a.z<b.z。
 */
template <typename PointType>
bool KD_TREE<PointType>::point_cmp_z(PointType a, PointType b) { return a.z < b.z; }

// Manual heap



// manual queue


// Manual Instatiations
template class KD_TREE<pcl::PointXYZ>;
template class KD_TREE<pcl::PointXYZI>;
template class KD_TREE<pcl::PointXYZINormal>;

