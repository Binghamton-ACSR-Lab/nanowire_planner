//
// Created by acsr on 4/26/21.
//

#ifndef NANOWIREPLANNER_TREE_NODE_HPP
#define NANOWIREPLANNER_TREE_NODE_HPP

#include <list>
#include <spatial/spatial.hpp>
#include <spatial/point_multiset.hpp>
#include <spatial/point_multimap.hpp>
#include <spatial/neighbor_iterator.hpp>
#include <spatial/metric.hpp>
#include <unordered_map>

namespace acsr {
/***
 * denote in which tree a node is
 */
    enum TreeId {
        forward = 0,
        reverse,
        connection
    };

/***
 * denote where the tree node
 */
    enum TreeNodeState {
        not_in_tree = 0,
        not_in_set,
        in_open_set,
        in_close_set,
        in_optimize_set
    };

    using TreeEdge =std::pair<Eigen::VectorXd,double>;

    class ProxNode;
    class SSTTreeNode;
    class TreeNode;
    class Node;
    using NodePtr = std::shared_ptr<Node>;
    using TreeNodePtr = std::shared_ptr <TreeNode>;
    using SSTTreeNodePtr = std::shared_ptr <SSTTreeNode>;
    using ProxNodePtr = std::shared_ptr <ProxNode>;


    class Node{
    protected:
        Eigen::VectorXd _state;
    public:
        /***
         * delete defaut constructor
         */
        Node() = delete;

        /***
         * costructor with an inital state
         * @param _state
         */
        explicit Node(const Eigen::VectorXd& state):_state(state){
        }

        /***
         * copy constructor
         * @param _node
         */
        Node(const Node& node){
            _state = node._state;
        }

        /***
         * override operator =
         * @param _node
         * @return
         */
        Node& operator=(const Node& node){
            if(&node == this)return *this;
            _state = node._state;
            return *this;
        }

        /***
         * override operator []
         * @param index
         * @return
         */
        double operator[](size_t index) const{
            if(index>_state.size())
                throw(std::out_of_range("index exceeds state range."));
            return _state[index];
        }

        /***
         * destructor
         */
        virtual ~Node()=default;

        /***
         * get the state
         * @return
         */
        virtual Eigen::VectorXd getState() const{
            return _state;
        }

    };

    class TreeNode: public Node {

    protected:
        std::weak_ptr <TreeNode> _parent;
        std::list<TreeNodePtr> _children;
        TreeEdge  _edge;
        double _cost;
        TreeId _tree_id;
        TreeNodeState _tree_node_state;


    public:
        /***
         * delete default constructor
         */
        TreeNode() = delete;

        /***
         * delete copy constructor
         */
        TreeNode(const TreeNode &) = delete;

        /***
         * delete assign operator
         * @return
         */
        TreeNode &operator=(const TreeNode &) = delete;

        /***
         * constructor
         * @param id: tree id
         * @param pt: node state
         */
        explicit TreeNode(TreeId id, const Eigen::VectorXd &pt):Node(pt),_tree_id(id),_cost(0.0),_tree_node_state(not_in_tree) {

        }

        /***
         * deconstructor
         */
        ~TreeNode() override {
            _children.clear();
        }

        /***
         * set the parent of this node
         * @param parent
         */
        void setParent(const TreeNodePtr &parent) {
            _parent = parent;
        }

        /***
         * get parent
         * @return
         */
        TreeNodePtr getParent() const {
            if (!_parent.expired())
                return _parent.lock();
            else
                return nullptr;
        }

        /***
         * set node  cost
         */
        void setCost(double cost) {
            _cost = cost;
        }

        /***
         * get node cost
         * @return
         */
        double getCost() const {
            return _cost;
        }

        /***
         * set the state of the node,e.g. where it is
         * @param nodeState
         */
        void setTreeNodeState(TreeNodeState nodeState) {
            _tree_node_state = nodeState;
        }

        /***
         * get the state of the node, e.g. where it is
         * @return
         */
        TreeNodeState getTreeNodeState() const {
            return _tree_node_state;
        }

        /***
         * add a child
         * @param child a child node
         */
        void addChild(const TreeNodePtr &child) {
            _children.emplace_back(child);
        }

        /***
         * remove a child
         * @param child
         */
        void removeChild(TreeNodePtr &child) {
            _children.remove(child);
        }

        /***
         * clean the children
         */
        void clearChildren() {
            _children.clear();
        }

        /***
         * get the children vector
         * @return
         */
        std::list <TreeNodePtr> getChildren() {
            return _children;
        }

        /***
         * get the tree id
         * @return
         */
        TreeId getTreeId() const {
            return _tree_id;
        }

        /***
         * set tree edge
         * @param _edge
         */
        void setEdge(const TreeEdge &edge) {
            _edge = edge;
        }

        /***
         * get the control of the edge
         * @return
         */
        Eigen::VectorXd getEdgeControl() const {
            return _edge.first;
        }

        /***
         * get the duration of the edge
         * @return
         */
        double getEdgeDuration() const {
            return _edge.second;
        }

        bool operator==(const TreeNode &node) const {
            return _state == node._state && _edge == node._edge && _tree_id == node._tree_id &&
                   _cost == node._cost;
        }

    };

    /*
    namespace std {
        class hash<TreeNode> {
        public:
            size_t operator()(const TreeNode &node) const {
                auto f = std::hash<Te>{};
                auto state = node.getState();
                std::size_t h = 0;
                for (auto i = 0; state.size(); ++i) {
                    h += f(state[i]);
                }
                h += std::hash<Te>{}(node.getCost());
                return h;
            }
        };
    }

    template<typename T, typename Te, typename Tc>
    struct TreeNodeHash {
        std::size_t operator()(const std::shared_ptr<TreeNode> &node) {
            auto state = node->getState();
            std::size_t h = 0;
            for (auto i = 0; state.size(); ++i) {
                h += std::hash<double>{}(state[i]);
            }
            h += std::hash<double>{}(node->getCost());
            return h;
        }

    };
*/
    class SSTTreeNode : public TreeNode{

    protected:
        ProxNodePtr _prox_node;
        bool _active_state;

    public:

        SSTTreeNode(/* args */) = delete;
        SSTTreeNode(const SSTTreeNode&) = delete;
        SSTTreeNode& operator=(const SSTTreeNode&) = delete;

        /***
         * inherits from treenode
         * @param id
         * @param pt
         */
        SSTTreeNode(TreeId id,const Eigen::VectorXd& pt): TreeNode(id,pt){

        }

        /***
         * set prox node
         * @param node
         */
        void setProxNode(ProxNodePtr node){
            _prox_node = node;
        }

        /***
         * get prox node
         * @return
         */
        ProxNodePtr getProxNode(){
            return _prox_node;
        }

        /***
         * check whether this sstnode active
         * @return
         */
        bool isActive(){
            return _active_state;
        }

        /***
         * set activeness
         * @param state
         */
        void setActive(bool state){
            _active_state = state;
        }

        /***
         * desctructor
         */
        ~SSTTreeNode() override =default;
    };


    class ProxNode : public Node{
    private:
        std::weak_ptr<SSTTreeNode> _monitor_node;

    public:
        ProxNode()= delete;

        /***
         * constructor, inherits from Node
         * @param _state
         */
        ProxNode(const Eigen::VectorXd& _state) : Node(_state){
        }

        /***
         * deconstructor
         */
        virtual ~ProxNode()= default;

        /***
         * set the monitor node
         * @param node
         */
        void setMonitorNode(std::shared_ptr<SSTTreeNode> node){
            _monitor_node = node;
        }

        /***
         * get the monitor node
         * @return
         */
        std::shared_ptr<SSTTreeNode> getMonitorNode(){
            if(!_monitor_node.expired())
                return _monitor_node.lock();
            else
                return nullptr;
        }
    };

    using KdTreeType = spatial::point_multimap<0, Eigen::VectorXd, std::shared_ptr<Node>>;
    //using KdProxTreeType = spatial::point_multimap<0, Eigen::VectorXd, std::shared_ptr<Node>>;
    using NodeMapType = std::unordered_map<std::shared_ptr<TreeNode>, double, std::hash<std::shared_ptr<TreeNode>>>;

}

#endif //NANOWIREPLANNER_TREE_NODE_HPP
