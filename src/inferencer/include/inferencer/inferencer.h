#ifndef INFERENCER__INFERENCER_H_
#define INFERENCER__INFERENCER_H_

#include <boost/function.hpp>

#include <dlfcn.h>

#include <string>

namespace inferencer
{
class Inferencer
{
public:
    virtual ~Inferencer() = default;

    virtual bool loadModel(const std::string& path) = 0;

    virtual unsigned getInputBuffer(const std::string& name, void** buffer) = 0;

    virtual unsigned getOutputBuffer(const std::string& name, void** buffer) = 0;

    virtual bool infer() = 0;

    virtual const std::string& getErrorString() const = 0;
};

}

#endif // INFERENCER__INFERENCER_H_
