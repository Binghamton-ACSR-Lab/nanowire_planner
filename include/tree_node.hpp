//
// Created by acsr on 4/26/21.
//

#ifndef NANOWIREPLANNER_TREE_NODE_HPP
#define NANOWIREPLANNER_TREE_NODE_HPP

#include <list>
//#include <spatial/spatial.hpp>
//#include <spatial/point_multiset.hpp>
//#include <spatial/point_multimap.hpp>
//#include <spatial/neighbor_iterator.hpp>
//#include <spatial/metric.hpp>
#include <unordered_map>

namespace acsr {
/***
 * denote in which tree a node is
 */
    enum TreeId {
        forward = 0,
        backward,
        connection
    };

/***
 * denote where the tree node
 */
 /*
    enum TreeNodeState {
        not_in_tree = 0,
        in_tree
    };
*/

    template <int CONTROL_DIMENSION>
    using TreeEdge =std::pair<Eigen::Matrix<double,CONTROL_DIMENSION,1>,double>;

    //template <int STATE_DIMENSION>
    //class ProxNode;

    template <int STATE_DIMENSION,int CONTROL_DIMENSION>
    class SSTTreeNode;

    template <int STATE_DIMENSION,int CONTROL_DIMENSION>
    class TreeNode;

    template <int STATE_DIMENSION>
    class Node;

    /*
    template <int STATE_DIMENSION>
    using NodePtr = std::shared_ptr<Node<STATE_DIMENSION>>;

    template <int STATE_DIMENSION,int CONTROL_DIMENSION>
    using TreeNodePtr = std::shared_ptr <TreeNode<STATE_DIMENSION,CONTROL_DIMENSION>>;

    template <int STATE_DIMENSION,int CONTROL_DIMENSION>
    using SSTTreeNodePtr = std::shared_ptr <SSTTreeNode<STATE_DIMENSION,CONTROL_DIMENSION>>;*/
    //using ProxNodePtr = std::shared_ptr <ProxNode>;

    template <int STATE_DIMENSION>
    class Node{
    protected:
        Eigen::Matrix<double,STATE_DIMENSION,1> _state;
    public:
        /***
         * delete defaut constructor
         */
        Node() = delete;

        /***
         * costructor with an inital state
         * @param _state
         */
        explicit Node(const Eigen::Matrix<double,STATE_DIMENSION,1>& state):_state(state){
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
        virtual Eigen::Matrix<double,STATE_DIMENSION,1> getState() const{
            return _state;
        }

    };

    template <int STATE_DIMENSION,int CONTROL_DIMENSION>
    class TreeNode: public Node<STATE_DIMENSION> {
        using TreeNodePtr = std::shared_ptr <TreeNode<STATE_DIMENSION,CONTROL_DIMENSION>>;
    protected:
        std::weak_ptr <TreeNode> _parent;
        std::list<TreeNodePtr> _children;
        TreeEdge<CONTROL_DIMENSION>  _edge;
        double _cost;
        TreeId _tree_id;
        //TreeNodeState _tree_node_state;


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
        explicit TreeNode(TreeId id, const Eigen::Matrix<double,STATE_DIMENSION,1> &pt):Node<STATE_DIMENSION>(pt),_tree_id(id),_cost(0.0) {

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

        void setTreeNodeState(TreeNodeState nodeState) {
            _tree_node_state = nodeState;
        }*/

        /***
         * get the state of the node, e.g. where it is
         * @return

        TreeNodeState getTreeNodeState() const {
            return _tree_node_state;
        }*/

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
        void setEdge(const TreeEdge<CONTROL_DIMENSION> &edge) {
            _edge = edge;
        }

        /***
         * get the control of the edge
         * @return
         */
        Eigen::Matrix<double,CONTROL_DIMENSION,1> getEdgeControl() const {
            return _edge.first;
        }

        /***
         * get the duration of the edge
         * @return
         */
        double getEdgeDuration() const {
            return _edge.second;
        }

        bool operator==(const TreeNode<STATE_DIMENSION,CONTROL_DIMENSION> &node) const {
            return this->_state == node._state && _edge == node._edge && _tree_id == node._tree_id &&
                   _cost == node._cost;
        }

    };

    template <int STATE_DIMENSION,int CONTROL_DIMENSION>
    class SSTTreeNode : public TreeNode<STATE_DIMENSION,CONTROL_DIMENSION>{

    protected:
        //ProxNodePtr _prox_node;
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
        SSTTreeNode(TreeId id,const Eigen::Matrix<double,STATE_DIMENSION,1>& pt): TreeNode<STATE_DIMENSION,CONTROL_DIMENSION>(id,pt){

        }

        /***
         * set prox node
         * @param node

        void setProxNode(ProxNodePtr node){
            _prox_node = node;
        }*/

        /***
         * get prox node
         * @return

        ProxNodePtr getProxNode(){
            return _prox_node;
        }*/

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

    //using KdTreeType = spatial::point_multimap<0, Eigen::VectorXd, std::shared_ptr<Node>>;

}

#endif //NANOWIREPLANNER_TREE_NODE_HPP
