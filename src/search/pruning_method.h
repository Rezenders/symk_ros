#ifndef PRUNING_METHOD_H
#define PRUNING_METHOD_H

#include "operator_id.h"

#include <memory>
#include <vector>

class AbstractTask;
class State;

namespace limited_pruning {
class LimitedPruning;
}

class PruningMethod {
    friend class limited_pruning::LimitedPruning;

    virtual void prune(
        const State &state, std::vector<OperatorID> &op_ids) = 0;
protected:
    std::shared_ptr<AbstractTask> task;
    long num_successors_before_pruning;
    long num_successors_after_pruning;
public:
    PruningMethod();
    virtual ~PruningMethod() = default;
    virtual void initialize(const std::shared_ptr<AbstractTask> &task);
    void prune_operators(const State &state, std::vector<OperatorID> &op_ids);
    virtual void print_statistics() const;
};

#endif
