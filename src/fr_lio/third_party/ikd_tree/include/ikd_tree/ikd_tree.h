#pragma once
#include <cmath>
#include <deque>
#include <memory>
#include <vector>
#include <pcl/point_types.h>

namespace ikd_tree {

struct ikdTree_PointType {
    float x, y, z;
    ikdTree_PointType(float px = 0.0f, float py = 0.0f, float pz = 0.0f) {
        x = px;
        y = py;
        z = pz;
    }
};

struct BoxPointType {
    float vertex_min[3];
    float vertex_max[3];
};

enum operation_set { ADD_POINT, DELETE_POINT, DELETE_BOX, ADD_BOX, DOWNSAMPLE_DELETE, PUSH_DOWN };

enum delete_point_storage_set { NOT_RECORD, DELETE_POINTS_REC, MULTI_THREAD_REC };

template <typename PointType> class KdTree {
  public:
    using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
    using Ptr = std::shared_ptr<KdTree<PointType>>;

  private:
    // Forward-declared private types; full definitions live in ikd_tree.cpp so
    // pthread.h is not part of the public header surface.
    struct Node;
    struct Threads;

    struct OperationLog {
        PointType point;
        BoxPointType boxpoint;
        bool tree_deleted, tree_downsample_deleted;
        operation_set op;
    };

    struct PointCmp {
        PointType point;
        float dist = 0.0;
        PointCmp(PointType p = PointType(), float d = INFINITY) {
            this->point = p;
            this->dist = d;
        };
        bool operator<(const PointCmp &a) const {
            if (fabs(dist - a.dist) < 1e-10)
                return point.x < a.point.x;
            else
                return dist < a.dist;
        }
    };

    class Heap {
      public:
        Heap(int max_capacity = 100) {
            cap = max_capacity;
            heap = new PointCmp[max_capacity];
            heap_size = 0;
        }

        ~Heap() { delete[] heap; }

        void pop() {
            if (heap_size == 0)
                return;
            heap[0] = heap[heap_size - 1];
            heap_size--;
            MoveDown(0);
            return;
        }

        PointCmp top() { return heap[0]; }

        void push(PointCmp point) {
            if (heap_size >= cap)
                return;
            heap[heap_size] = point;
            FloatUp(heap_size);
            heap_size++;
            return;
        }

        int size() { return heap_size; }

        void clear() { heap_size = 0; }

      private:
        int heap_size = 0;
        int cap = 0;
        PointCmp *heap;
        void MoveDown(int heap_index) {
            int l = heap_index * 2 + 1;
            PointCmp tmp = heap[heap_index];
            while (l < heap_size) {
                if (l + 1 < heap_size && heap[l] < heap[l + 1])
                    l++;
                if (tmp < heap[l]) {
                    heap[heap_index] = heap[l];
                    heap_index = l;
                    l = heap_index * 2 + 1;
                } else
                    break;
            }
            heap[heap_index] = tmp;
            return;
        }

        void FloatUp(int heap_index) {
            int ancestor = (heap_index - 1) / 2;
            PointCmp tmp = heap[heap_index];
            while (heap_index > 0) {
                if (heap[ancestor] < tmp) {
                    heap[heap_index] = heap[ancestor];
                    heap_index = ancestor;
                    ancestor = (heap_index - 1) / 2;
                } else
                    break;
            }
            heap[heap_index] = tmp;
            return;
        }
    };

    // Multi-thread Tree Rebuild
    bool termination_flag = false;
    bool rebuild_flag = false;
    std::unique_ptr<Threads> threads_;
    std::deque<OperationLog> Rebuild_Logger;
    PointVector Rebuild_PCL_Storage;
    Node **Rebuild_Ptr = nullptr;
    int search_mutex_counter = 0;
    static void *multi_thread_ptr(void *arg);
    void multi_thread_rebuild();
    void start_thread();
    void stop_thread();
    void run_operation(Node **root, const OperationLog &operation);
    // KD Tree Functions and augmented variables
    int Treesize_tmp = 0, Validnum_tmp = 0;
    float alpha_bal_tmp = 0.5, alpha_del_tmp = 0.0;
    float delete_criterion_param = 0.5f;
    float balance_criterion_param = 0.7f;
    float downsample_size = 0.2f;
    bool Delete_Storage_Disabled = false;
    Node *STATIC_ROOT_NODE = nullptr;
    PointVector Points_deleted;
    PointVector Downsample_Storage;
    PointVector Multithread_Points_deleted;
    void InitTreeNode(Node *root);
    void Test_Lock_States(Node *root);
    void BuildTree(Node **root, int l, int r, PointVector &Storage);
    void Rebuild(Node **root);
    int Delete_by_range(Node **root, BoxPointType boxpoint, bool allow_rebuild, bool is_downsample);
    void Delete_by_point(Node **root, const PointType &point, bool allow_rebuild);
    void Add_by_point(Node **root, const PointType &point, bool allow_rebuild, int father_axis);
    void Add_by_range(Node **root, BoxPointType boxpoint, bool allow_rebuild);
    void Search(Node *root, int k_nearest, const PointType &point, Heap &q, double max_dist);
    void Search_by_range(Node *root, BoxPointType boxpoint, PointVector &Storage);
    void Search_by_radius(Node *root, const PointType &point, float radius, PointVector &Storage);
    bool Criterion_Check(Node *root);
    void Push_Down(Node *root);
    void Update(Node *root);
    void delete_tree_nodes(Node **root);
    void downsample(Node **root);
    inline bool same_point(const PointType &a, const PointType &b);
    inline float calc_dist(const PointType &a, const PointType &b);
    inline float calc_box_dist(Node *node, const PointType &point);
    static inline bool point_cmp_x(PointType a, PointType b);
    static inline bool point_cmp_y(PointType a, PointType b);
    static inline bool point_cmp_z(PointType a, PointType b);

    void flatten(Node *root, PointVector &Storage, delete_point_storage_set storage_type);

    PointVector PCL_Storage;
    Node *Root_Node = nullptr;

  public:
    KdTree(float delete_param = 0.5, float balance_param = 0.7, float box_length = 0.2);
    ~KdTree();
    void set_delete_criterion_param(float delete_param);
    void set_balance_criterion_param(float balance_param);
    void set_downsample_param(float box_length);
    void initialize(float delete_param = 0.5, float balance_param = 0.7, float box_length = 0.2);
    int size();
    int validnum();
    void root_alpha(float &alpha_bal, float &alpha_del);
    void build(PointVector point_cloud);
    void nearest_search(const PointType &point, int k_nearest, PointVector &Nearest_Points,
                        std::vector<float> &Point_Distance, double max_dist = INFINITY);
    void box_search(const BoxPointType &Box_of_Point, PointVector &Storage);
    void radius_search(const PointType &point, const float radius, PointVector &Storage);
    int add_points(PointVector &PointToAdd, bool downsample_on);
    void add_point_boxes(std::vector<BoxPointType> &BoxPoints);
    int delete_points(PointVector &PointToDel);
    int delete_point_boxes(std::vector<BoxPointType> &BoxPoints);
    void flatten(PointVector &Storage);
    void acquire_removed_points(PointVector &removed_points);
    BoxPointType tree_range();

  private:
    int max_queue_size = 0;
};

} // namespace ikd_tree
