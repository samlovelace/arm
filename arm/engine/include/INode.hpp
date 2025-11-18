#ifndef INODE_HPP
#define INODE_HPP
 
#include <memory> 

class INode 
{ 
public:

    virtual ~INode() = default; 

    enum class Status
    {
        RUNNING, 
        SUCCESS, 
        FAILURE, 
        NUM_TYPES
    }; 

    virtual Status tick() = 0; 

protected:
   
};

// cleaner type def 
using NodePtr = std::shared_ptr<INode>; 

#endif //INODE_HPP