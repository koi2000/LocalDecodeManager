# MCGAL-CUDA
尝试将MCGAL迁移至CUDA上进行计算

首先考虑成员变量
vertex中含有halfedge的数组

halfedge中有两个vertex，一个opposite，一个face

face中有一个vertex数组，一个halfedge数组


当前的初步想法：

将所有的对象全部池化，目前已经在cpu版本实现，后面会迁移至gpu版本，在gpu上实现池化操作

首先创建三个对象池，vertex，halfedge，face。

创建对象的过程即为从对象池中拿对象，然后将池的index++（原子操作）即可，最后由池统一释放内存

目前版本没有使用任何复杂的数据结构，全部使用vector管理，为了迁移至GPU需要将指针属性移除，转为维护一个index，这个index就是对象在池中的index

可以使用一个int来代表一个对应的数据结构，在mesh中新增点，新增面，
此时需要考虑的另一个问题是，动态的新增点和面，mesh中由vertex和face数组，vertex中也有halfedge，在decode过程中，他们需要进行新增操作，
目前能想到的有两种大致的解决思路：

1. 开一个较大的数组，维护一个size
2. 动态的分配，实现一个简易版的vector，这里要考虑的一个点是 