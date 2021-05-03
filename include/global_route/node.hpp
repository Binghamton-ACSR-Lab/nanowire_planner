//
// Created by acsr on 4/5/21.
//

#ifndef TRANSITIONMATRIXROUTE_NODE_HPP
#define TRANSITIONMATRIXROUTE_NODE_HPP
#include <memory>
#include <unordered_map>
#include <acado/external_packages/eigen3/Eigen/Eigen>
#include <numeric>


namespace acsr {
    namespace global {
        using NanowirePositionType = std::vector<std::pair<int, int>>;
        using IndexType = int64_t;
        using ControlType = uint32_t;

        /**
         * convert int64 index to vector of position
         * @param n_wires wire count
         * @param index index
         * @return vector of position
         */
        NanowirePositionType indexToElectrodeVector(int n_wires, IndexType index) {
            NanowirePositionType electrode_vector(n_wires);
            for (auto i = 0; i < n_wires; ++i) {
                auto v = index & 0b1111;
                electrode_vector[n_wires - i - 1] = std::make_pair(int(v >> 2), int(v & 0b11));
                index = index >> 4;
            }
            return electrode_vector;
        }

        /**
         * index to divided indice
         * @param index index
         * @param divided_vec nanowire divided vector
         * @return sub indice vector
         */
        std::vector<IndexType> indexToSubIndexVec(IndexType index, const std::vector<int> &divided_vec) {
            std::vector<IndexType> sub_node_index(divided_vec.size());
            for (auto i = divided_vec.size() - 1; i > 0; --i) {
                IndexType v = (1 << 4 * divided_vec[i]) - 1;
                sub_node_index[i] = index & v;
                index = index >> 4 * divided_vec[i];
            }
            sub_node_index[0] = (index);
            return sub_node_index;
        }

        /**
         * position vector to index
         * @param n_wires wire count
         * @param electrode_vector position vector
         * @return index (int64 type)
         */
        IndexType electrodeVectorToIndex(int n_wires, const NanowirePositionType &electrode_vector) {
            IndexType index = (electrode_vector[0].first << 2) + electrode_vector[0].second;
            for (auto i = 1; i < n_wires; ++i)
                index = (index << 4) + (electrode_vector[i].first << 2) + electrode_vector[i].second;
            return index;
        }

        /**
         * combine sub indice vector to an index
         * @param sub_index_vec sub index vec
         * @param divided_vec nanowire divided vector
         * @return index
         */
        IndexType subIndexVecToIndex(const std::vector<IndexType> &sub_index_vec, const std::vector<int> &divided_vec) {
            auto index = sub_index_vec[0];
            for (auto i = 1; i < divided_vec.size(); ++i)
                index = index << (4 * divided_vec[i]) | sub_index_vec[i];
            return index;
        }


        /**
         * control column type
         */
        struct ControlVectorType {
        public:

            /**
             * default constructor
             */
            ControlVectorType() = default;

            /**
             * constructor
             * @param size
             */
            explicit ControlVectorType(IndexType size) : _size(size) {

            }

            /**
             * set data
             * @param data data
             */
            void setData(const std::vector<std::pair<IndexType, ControlType>> &data) {
                vec = data;
                //_element_size = data.size();
            }

            /**
             * set size
             * @param size
             */
            void setSize(IndexType size) {
                _size = size;
            }

            void setData(std::vector<std::pair<IndexType, ControlType>> &&data) {
                vec = std::move(data);
            }

            /*
             * get indice vector
             */
            std::vector<IndexType> getIndexVector() {
                std::vector<IndexType> v(vec.size());
                std::transform(vec.begin(), vec.end(), v.begin(), [](const std::pair<IndexType, ControlType> &p) {
                    return p.first;
                });
                return v;
            }

            std::vector<std::pair<IndexType, ControlType>> vec{};
            IndexType _size{};
        };

        /**
         * tree node
         */
        class TransitionTreeNode : public std::enable_shared_from_this<TransitionTreeNode> {
        public:
            /**
             * default construtor
             */
            TransitionTreeNode() = default;

            /**
             * constructor
             * @param _state state
             * @param _level level
             */
            TransitionTreeNode(IndexType _state, int _level) : state(_state), level(_level) {}

            /**
             * default deconstructor
             */
            virtual ~TransitionTreeNode() = default;

            /**
             * copy constructor
             */
            TransitionTreeNode(const TransitionTreeNode &) = delete;

            /*
             * assign
             */
            TransitionTreeNode &operator=(const TransitionTreeNode &) = delete;

            /**
             * create a add a child node and insert
             * @param child_index  child index
             * @param _level child level
             * @return child node ptr
             */
            std::shared_ptr<TransitionTreeNode> addChild(IndexType child_index, int _level) {
                auto child = std::make_shared<TransitionTreeNode>(child_index, _level);
                children.push_back(child);
                return child;
            }

            /**
             * add an existing node as child
             * @param child child ptr
             */
            void addChild(const std::shared_ptr<TransitionTreeNode> &child) {
                children.push_back(child);
            }

            /**
             * get state
             * @return
             */
            IndexType getState() const {
                return state;
            }

            /**
             * get level
             * @return
             */
            int getLevel() const {
                return level;
            }

            /**
             * set level
             * @param value new level
             */
            void setLevel(int value) {
                level = value;
            }

            /**
             * set parent node
             * @param _parent parent node
             */
            void setParent(const std::shared_ptr<TransitionTreeNode> &_parent) {
                parent = _parent;
            }

            /**
             * get parent node
             * @return
             */
            std::shared_ptr<TransitionTreeNode> getParent() {
                if (parent.expired()) {
                    return nullptr;
                } else {
                    return parent.lock();
                }
            }

            /**
             * remove a child
             * @param child
             */
            void removeChild(std::shared_ptr<TransitionTreeNode> child) {
                children.remove(child);
            }

            /**
             * get all children
             * @return children list
             */
            std::list<std::shared_ptr<TransitionTreeNode>> &getChildren() {
                return children;
            }

            /**
             * set state quality
             * @param value qualilty
             */
            void setPathQuality(int value) {
                path_quality = value;
            }

            /**
             * get state quality
             * @return
             */
            int getPathQuality() const {
                return path_quality;
            }


        protected:
            ///state
            IndexType state{};

            ///tree level
            int level{};

            /// state quality
            int path_quality{};

            ///parent node
            std::weak_ptr<TransitionTreeNode> parent;

            ///children node list
            std::list<std::shared_ptr<TransitionTreeNode>> children;

        };

        class AstarNode : public TransitionTreeNode {
        public:
            AstarNode(IndexType _state, int _level, bool _node_state) : TransitionTreeNode(_state, _level),
                                                                        node_state(_node_state) {}

            void setNodeState(bool state) {
                node_state = state;
            }

            bool getNodeState() const {
                return node_state;
            }

        protected:
            bool node_state; //true: open set; false: close set;
        };


        using NodePtr = std::shared_ptr<TransitionTreeNode>;
    }
}



#endif //TRANSITIONMATRIXROUTE_NODE_HPP
