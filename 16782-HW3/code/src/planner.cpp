#include <iostream>
#include <fstream>
#include <regex>
#include <unordered_set>
#include <set>
#include <list>
#include <unordered_map>
#include <algorithm>
#include <stdexcept>
#include <queue>
#include <limits>
#include <chrono>


#define SYMBOLS 0
#define INITIAL 1
#define GOAL 2
#define ACTIONS 3
#define ACTION_DEFINITION 4
#define ACTION_PRECONDITION 5
#define ACTION_EFFECT 6
#ifndef ENVS_DIR
#define ENVS_DIR "../envs"
#endif
class GroundedCondition;
class Condition;
class GroundedAction;
class Action;
class Env;

using namespace std;

bool print_status = true;

// predicate: Clear, On
// arg_values: [A], [A, B]
class GroundedCondition
{
    string predicate;
    list<string> arg_values;
    bool truth = true;

public:
    GroundedCondition(const string &predicate, const list<string> &arg_values, bool truth = true)
    {
        this->predicate = predicate;
        this->truth = truth;  // fixed
        for (const string& l : arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    GroundedCondition(const GroundedCondition& gc)
    {
        this->predicate = gc.predicate;
        this->truth = gc.truth;  // fixed
        for (const string& l : gc.arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }
    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    void set_truth(bool truth)
    {
        this->truth = truth;
    }

    friend ostream& operator<<(ostream& os, const GroundedCondition& pred)
    {
        os << pred.toString() << " ";
        return os;
    }

    bool operator==(const GroundedCondition& rhs) const
    {
        if (this->predicate != rhs.predicate || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth()) // fixed
            return false;

        return true;
    }

    string toString() const
    {
        string temp;
        if (!this->truth)
            temp += "!";
        temp += this->predicate;
        temp += "(";
        for (const string& l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct GroundedConditionHasher
{
    size_t operator()(const GroundedCondition& gcond) const
    {
        return hash<string>{}(gcond.toString());
    }
};

struct GroundedConditionComparator
{
    bool operator()(const GroundedCondition& lhs, const GroundedCondition& rhs) const
    {
        return lhs == rhs;
    }
};

using GroundedConditionSet = unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>;

class Condition
{
    string predicate;
    list<string> args;
    bool truth;

public:
    Condition(const string &pred, const list<string>& args, const bool truth)
    {
        this->predicate = pred;
        this->truth = truth;
        for (const string& ar : args)
        {
            this->args.push_back(ar);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }

    list<string> get_args() const
    {
        return this->args;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    friend ostream& operator<<(ostream& os, const Condition& cond)
    {
        os << cond.toString() << " ";
        return os;
    }

    bool operator==(const Condition& rhs) const // fixed
    {

        if (this->predicate != rhs.predicate || this->args.size() != rhs.args.size())
            return false;

        auto lhs_it = this->args.begin();
        auto rhs_it = rhs.args.begin();

        while (lhs_it != this->args.end() && rhs_it != rhs.args.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth())
            return false;

        return true;
    }

    string toString() const
    {
        string temp;
        if (!this->truth)
            temp += "!";
        temp += this->predicate;
        temp += "(";
        for (const string& l : this->args)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ConditionHasher
{
    size_t operator()(const Condition& cond) const
    {
        return hash<string>{}(cond.toString());
    }
};

struct ConditionComparator
{
    bool operator()(const Condition& lhs, const Condition& rhs) const
    {
        return lhs == rhs;
    }
};

using ConditionSet = unordered_set<Condition, ConditionHasher, ConditionComparator>;

// Example: GroundedAction("MoveToTable", { "A", "B" })
class GroundedAction
{
    string name;
    list<string> arg_values;
    GroundedConditionSet preconditions;
    GroundedConditionSet effects;

public:
    GroundedAction(const string &name, const list<string>& arg_values,
                    const GroundedConditionSet& preconditions,
                    const GroundedConditionSet& effects)
    {
        this->name = name;
        for (const string& ar : arg_values)
        {
            this->arg_values.push_back(ar);
        }
        for (const GroundedCondition& pc : preconditions)
        {
            this->preconditions.insert(pc);
        }
        for (const GroundedCondition& pc : effects)
        {
            this->effects.insert(pc);
        }
    }

    string get_name() const
    {
        return this->name;
    }
    list<string> get_arg_values() const
    {
        return this->arg_values;
    }
    GroundedConditionSet get_preconditions() const
    {
        return this->preconditions;
    }
    GroundedConditionSet get_effects() const
    {
        return this->effects;
    }

    bool operator==(const GroundedAction& rhs) const
    {
        if (this->name != rhs.name || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }
        return true;
    }

    friend ostream& operator<<(ostream& os, const GroundedAction& gac)
    {
        os << gac.toString() << " ";
        os << "Precondition: ";
        for (const GroundedCondition& precond : gac.get_preconditions())
            os << precond;
        os << endl;
        os << "Effect: ";
        for (const GroundedCondition& effect : gac.get_effects())
            os << effect;
        os << endl;
        return os;
    }

    string toString() const
    {
        string temp;
        temp += this->name;
        temp += "(";
        for (const string& l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct GroundedActionHasher
{
    size_t operator()(const GroundedAction& gac) const
    {
        return hash<string>{}(gac.toString());
    }
};

struct GroundedActionComparator
{
    bool operator()(const GroundedAction& lhs, const GroundedAction& rhs) const
    {
        return lhs == rhs;
    }
};

using GroundedActionSet = unordered_set<GroundedAction, GroundedActionHasher, GroundedActionComparator>;

class Action
{
    string name;
    list<string> args;
    ConditionSet preconditions;
    ConditionSet effects;

public:
    Action(const string &name, const list<string>& args,
           const ConditionSet& preconditions,
           const ConditionSet& effects)
    {
        this->name = name;
        for (const string& l : args)
        {
            this->args.push_back(l);
        }
        for (const Condition& pc : preconditions)
        {
            this->preconditions.insert(pc);
        }
        for (const Condition& pc : effects)
        {
            this->effects.insert(pc);
        }
    }
    string get_name() const
    {
        return this->name;
    }
    list<string> get_args() const
    {
        return this->args;
    }
    ConditionSet get_preconditions() const
    {
        return this->preconditions;
    }
    ConditionSet get_effects() const
    {
        return this->effects;
    }

    bool operator==(const Action& rhs) const
    {
        if (this->get_name() != rhs.get_name() || this->get_args().size() != rhs.get_args().size())
            return false;

        return true;
    }

    friend ostream& operator<<(ostream& os, const Action& ac)
    {
        os << ac.toString() << endl;
        os << "Precondition: ";
        for (const Condition& precond : ac.get_preconditions())
            os << precond;
        os << endl;
        os << "Effect: ";
        for (const Condition& effect : ac.get_effects())
            os << effect;
        os << endl;
        return os;
    }

    string toString() const
    {
        string temp;
        temp += this->get_name();
        temp += "(";
        for (const string& l : this->get_args())
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ActionHasher
{
    size_t operator()(const Action& ac) const
    {
        return hash<string>{}(ac.get_name());
    }
};

struct ActionComparator
{
    bool operator()(const Action& lhs, const Action& rhs) const
    {
        return lhs == rhs;
    }
};

using ActionSet = unordered_set<Action, ActionHasher, ActionComparator>;

class Env
{
    GroundedConditionSet initial_conditions;
    GroundedConditionSet goal_conditions;
    ActionSet actions;
    unordered_set<string> symbols;

public:
    void remove_initial_condition(const GroundedCondition& gc)
    {
        this->initial_conditions.erase(gc);
    }
    void add_initial_condition(const GroundedCondition& gc)
    {
        this->initial_conditions.insert(gc);
    }
    void add_goal_condition(const GroundedCondition& gc)
    {
        this->goal_conditions.insert(gc);
    }
    void remove_goal_condition(const GroundedCondition& gc)
    {
        this->goal_conditions.erase(gc);
    }
    void add_symbol(const string& symbol)
    {
        symbols.insert(symbol);
    }
    void add_symbols(const list<string>& symbols)
    {
        for (const string& l : symbols)
            this->symbols.insert(l);
    }
    void add_action(const Action& action)
    {
        this->actions.insert(action);
    }

    Action get_action(const string& name) const {
        for (Action a : this->actions)
        {
            if (a.get_name() == name)
                return a;
        }
        throw runtime_error("Action " + name + " not found!");
    }

    unordered_set<string> get_symbols() const
    {
        return this->symbols;
    }

    friend ostream& operator<<(ostream& os, const Env& w)
    {
        os << "***** Environment *****" << endl << endl;
        os << "Symbols: ";
        for (const string& s : w.get_symbols())
            os << s + ",";
        os << endl;
        os << "Initial conditions: ";
        for (const GroundedCondition& s : w.initial_conditions)
            os << s;
        os << endl;
        os << "Goal conditions: ";
        for (const GroundedCondition& g : w.goal_conditions)
            os << g;
        os << endl;
        os << "Actions:" << endl;
        for (const Action& g : w.actions)
            os << g << endl;
        cout << "***** Environment Created! *****" << endl;
        return os;
    }

    // add getters
    auto get_initial_conditions() const
    {
        return this->initial_conditions;
    }
    auto get_goal_conditions() const
    {
        return this->goal_conditions;
    }
    auto get_actions() const
    {
        return this->actions;
    }
};

list<string> parse_symbols(string symbols_str)
{
    list<string> symbols;
    size_t pos = 0;
    string delimiter = ",";
    while ((pos = symbols_str.find(delimiter)) != string::npos)
    {
        string symbol = symbols_str.substr(0, pos);
        symbols_str.erase(0, pos + delimiter.length());
        symbols.push_back(symbol);
    }
    symbols.push_back(symbols_str);
    return symbols;
}

Env* create_env(char* filename)
{
    ifstream input_file(filename);
    Env* env = new Env();
    regex symbolStateRegex("symbols:", regex::icase);
    regex symbolRegex("([a-zA-Z0-9_, ]+) *");
    regex initialConditionRegex("initialconditions:(.*)", regex::icase);
    regex conditionRegex("(!?[A-Z][a-zA-Z_]*) *\\( *([a-zA-Z0-9_, ]+) *\\)");
    regex goalConditionRegex("goalconditions:(.*)", regex::icase);
    regex actionRegex("actions:", regex::icase);
    regex precondRegex("preconditions:(.*)", regex::icase);
    regex effectRegex("effects:(.*)", regex::icase);
    int parser = SYMBOLS;

    ConditionSet preconditions;
    ConditionSet effects;
    string action_name;
    string action_args;

    string line;
    if (input_file.is_open())
    {
        while (getline(input_file, line))
        {
            string::iterator end_pos = remove(line.begin(), line.end(), ' ');
            line.erase(end_pos, line.end());

            if (line.empty())
                continue;

            if (parser == SYMBOLS)
            {
                smatch results;
                if (regex_search(line, results, symbolStateRegex))
                {
                    line = line.substr(8);
                    sregex_token_iterator iter(line.begin(), line.end(), symbolRegex, 0);
                    sregex_token_iterator end;

                    env->add_symbols(parse_symbols(iter->str()));  // fixed

                    parser = INITIAL;
                }
                else
                {
                    cout << "Symbols are not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == INITIAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, initialConditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_initial_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_initial_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = GOAL;
                }
                else
                {
                    cout << "Initial conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == GOAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, goalConditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_goal_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_goal_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = ACTIONS;
                }
                else
                {
                    cout << "Goal conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTIONS)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, actionRegex))
                {
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Actions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_DEFINITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, conditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;
                    // name
                    action_name = iter->str();
                    iter++;
                    // args
                    action_args = iter->str();
                    iter++;

                    parser = ACTION_PRECONDITION;
                }
                else
                {
                    cout << "Action not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_PRECONDITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, precondRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition precond(predicate, parse_symbols(args), truth);
                        preconditions.insert(precond);
                    }

                    parser = ACTION_EFFECT;
                }
                else
                {
                    cout << "Precondition not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_EFFECT)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, effectRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition effect(predicate, parse_symbols(args), truth);
                        effects.insert(effect);
                    }

                    env->add_action(
                        Action(action_name, parse_symbols(action_args), preconditions, effects));

                    preconditions.clear();
                    effects.clear();
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Effects not specified correctly." << endl;
                    throw;
                }
            }
        }
        input_file.close();
    }

    else
        cout << "Unable to open file";

    return env;
}


struct Node 
{
    int g = numeric_limits<int>::max();
    int h = numeric_limits<int>::max();
    int f = numeric_limits<int>::max();

    // parent (condition), a node
    Node* parent = nullptr;

    // action (parent to this node), an edge
    GroundedAction* parentAction = nullptr;

    // conditions
    GroundedConditionSet conditions;

    // constant reference, no copy, no modification
    Node (const GroundedConditionSet& conds)
    {
        this->conditions = conds;
    }

    void calHeuristic(const Env* env)
    {
        // this->h = 0;
        int result = 0;
        for (const GroundedCondition& gc : env->get_goal_conditions())
        {
            if (this->conditions.find(gc) == this->conditions.end())
            {
                result++;
            }
        }
        this->h = result;
    }

    void updatePriority()
    {
        this->f = this->g + this->h;
    }

    string toString() const
    {
        string temp;
        for (const GroundedCondition& gc : this->conditions)
        {
            temp += gc.toString() + " ";
        }
        return temp;
    }

};

struct CompareNode
{
    bool operator()(const Node* lhs, const Node* rhs) const
    {
        return lhs->f > rhs->f;
    }
};

bool checkConditions(const GroundedConditionSet& aConds, const GroundedConditionSet& bConds) {
    for (const GroundedCondition& condition : aConds) {
        // Check if the condition exists in bConds
        if (bConds.find(condition) == bConds.end()) {
            // If the condition is not found, check its truth value
            if (condition.get_truth()) {
                // If the condition is true and not found, return false
                return false;
            }
        }
    }
    return true;
}

bool checkGoalConditions(const GroundedConditionSet& aConds, const GroundedConditionSet& goalConds) {
    for (const GroundedCondition& condition : goalConds) {
        if (aConds.find(condition) == aConds.end()) {
            return false;
        }
    }
    return true;
}

GroundedConditionSet applyAction(Node* node, const GroundedAction& ga)
{
    // cout << "applyAction" << endl;
    GroundedConditionSet newConds = node->conditions;
    for (const GroundedCondition& effect : ga.get_effects())
    {
        if (effect.get_truth())
        {
            newConds.insert(effect);
        }
        else
        {
            GroundedCondition tempEffect = effect;
            // cout << "tempEffect: " << tempEffect << endl;
            tempEffect.set_truth(true);
            newConds.erase(tempEffect);
        }
    }
    return newConds;
}


stack<Node*> aStar(const GroundedConditionSet& initConds,
                    const GroundedConditionSet& goalConds,
                    const unordered_map<string, list<GroundedAction>>& actionSpace,
                    const Env* env)
{
    cout << "A* Search" << endl;

    stack<Node*> path;

    // open list
    priority_queue<Node*, vector<Node*>, CompareNode> open;
    // closed list
    list<GroundedConditionSet> closed;

    // start node
    Node* start = new Node(initConds);
    start->g = 0;
    start->calHeuristic(env);
    start->updatePriority();
    start->parent = nullptr;
    start->parentAction = nullptr;
    
    open.push(start);

    // cout << "Start Node: " << start << endl;
    while (!open.empty())
    {
        Node* topPriorityNode = open.top();
        open.pop();

        // cout << "topPriorityNode: " << topPriorityNode->toString() << endl;

        if (checkGoalConditions(topPriorityNode->conditions, goalConds))
        {
            // goal reached, backtrace
            while (topPriorityNode != nullptr)
            {
                path.push(topPriorityNode);
                topPriorityNode = topPriorityNode->parent;
            }
            cout << "Explored Node Num: " << closed.size() << endl;
            return path;
        }

        closed.push_back(topPriorityNode->conditions);

        // get all possible actions
        for (auto& aPair : actionSpace)
        {
            string generalActionName = aPair.first;
            for (const GroundedAction& ga : aPair.second)
            {
                if (checkConditions(ga.get_preconditions(), topPriorityNode->conditions))
                {
                    // cout << "Action: " << ga.toString() << endl;
                    GroundedConditionSet newConditions = applyAction(topPriorityNode, ga);

                    // check if the new conditions are in the closed list
                    bool inClosed = false;
                    for (const GroundedConditionSet& gcSet : closed)
                    {
                        if (checkConditions(newConditions, gcSet))
                        {
                            inClosed = true;
                            break;
                        }
                    }
                    if (!inClosed) 
                    {
                        Node* newNode = new Node(newConditions);
                        newNode->parent = topPriorityNode;
                        newNode->parentAction = new GroundedAction(ga);
                        newNode->g = topPriorityNode->g + 1;
                        newNode->calHeuristic(env);
                        newNode->updatePriority();
                        open.push(newNode);
                    }
                    else
                    {
                        // cout << "In closed" << endl;
                    }
                }
            }
        }

    }
    return path;
}

void groundCondition(const ConditionSet& conditions,
                        const list<string>& args,
                        const list<string>& params,
                        GroundedConditionSet& grounded_conds)
{
    // turn every conditions into grounded conditions
    // turn the general action args to params
    // args might exist symbol that is in the params, we only replace the ones that are not in the params

    // directly modify the grounded_conds
    unordered_map<string, string> argMap;
    // sanity check, they should always be of the same size
    // cout << "Args: " << args.size() << ", Params: " << params.size() << endl;
    auto paramIter = params.begin();
    for (auto argIter = args.begin(); argIter != args.end() && paramIter != params.end(); ++argIter, ++paramIter) {
        argMap[*argIter] = *paramIter;
    }

    for (const Condition& cond : conditions)
    {
        list<string> groundedArgs;
        for (const string& arg : cond.get_args())
        {
            if (argMap.count(arg) > 0) {
                groundedArgs.push_back(argMap[arg]);
            } else {
                // If the condition argument is not in the map, keep its original value
                groundedArgs.push_back(arg);
            }
        }
        grounded_conds.insert(GroundedCondition(cond.get_predicate(), groundedArgs, cond.get_truth()));
    }
}

// Helper function to generate all permutations of a combination
void generatePermutations(const list<string>& combination, list<list<string>>& combinations) {
    vector<string> temp(combination.begin(), combination.end());
    do {
        combinations.push_back(list<string>(temp.begin(), temp.end()));
    } while (next_permutation(temp.begin(), temp.end()));
}

// Function to generate the action space with all grounded actions
unordered_map<string, list<GroundedAction>> generateActionSpace(const ActionSet& actions, const unordered_set<string>& symbols)
{
    unordered_map<string, list<GroundedAction>> actionSpace;

    list<string> symbolList(symbols.begin(), symbols.end());
    // Ensure consistent ordering
    symbolList.sort();

    // Iterate over each action template
    for (const Action& action : actions) {
        auto paramCount = action.get_args().size();
        // Skip if not enough symbols for parameters
        if (paramCount > symbolList.size()) {
            // should not happen
            cout << "Not enough symbols for parameters" << endl;
            continue;
        }

        // Generate unique combinations of symbols
        vector<string> symbolVector(symbolList.begin(), symbolList.end());

        // select = {true, true, true};
        vector<bool> select(paramCount, true);
        // select = {true, true, true, false, false};
        select.resize(symbolVector.size(), false);

        list<list<string>> parameterCombinations;
        do {
            list<string> combination;
            for (size_t i = 0; i < symbolVector.size(); ++i) {
                if (select[i]) {
                    combination.push_back(symbolVector[i]);
                }
            }
            // Generate all permutations for each unique combination
            generatePermutations(combination, parameterCombinations);
        } while (prev_permutation(select.begin(), select.end()));

        // Add each grounded action to the action space
        for (const auto& params : parameterCombinations) {
            GroundedConditionSet groundedPreconditions;
            GroundedConditionSet groundedEffects;
            groundCondition(action.get_preconditions(), action.get_args(), params, groundedPreconditions);
            groundCondition(action.get_effects(), action.get_args(), params, groundedEffects);
            GroundedAction groundedAction(action.get_name(), params, groundedPreconditions, groundedEffects);
            actionSpace[action.get_name()].push_back(groundedAction);
        }
    }
    return actionSpace;
}

list<GroundedAction> planner(Env* env)
{
    //////////////////////////////////////////
    ///// TODO: INSERT YOUR PLANNER HERE /////
    //////////////////////////////////////////

    auto startTime = chrono::high_resolution_clock::now();
    list<GroundedAction> plan;

    ActionSet actions = env->get_actions();
    unordered_set<string> symbols = env->get_symbols();

    // Have a bunch of actions, now need to generate all possible combinations of actions with the symbols
    unordered_map<string, list<GroundedAction>> actionSpace = generateActionSpace(actions, symbols);

    // for (auto& aPair : actionSpace) {
    //     cout << "Action: " << aPair.first << endl;
    //     for (const GroundedAction& ga : aPair.second) {
    //         cout << ga << endl;
    //     }
    // }

    // start
    GroundedConditionSet initialConditions = env->get_initial_conditions();
    // cout << "Initial Conditions: " << endl;
    // for (const GroundedCondition& gc : initialConditions)
    // {
    //     cout << gc << endl;
    // }

    // goal
    GroundedConditionSet goalConditions = env->get_goal_conditions();
    // cout << "Goal Conditions: " << endl;
    // for (const GroundedCondition& gc : goalConditions)
    // {
    //     cout << gc << endl;
    // }

    // top: start, bottom: goal
    stack<Node*> path = aStar(initialConditions, goalConditions, actionSpace, env);
    if (path.empty())
    {
        cout << "No path found! Return Empth Plan" << endl;
        return plan;
    }

    while (!path.empty())
    {
        Node* cur = path.top();
        path.pop();
        if (cur->parentAction != nullptr)
        {
            plan.push_back(*(cur->parentAction));
        }
    }

    // Blocks World example (TODO: CHANGE THIS)
    // cout << endl << "CREATING DEFAULT PLAN" << endl;
    // list<GroundedAction> actions;
    // plan.push_back(GroundedAction("MoveToTable", { "A", "B" }));
    // plan.push_back(GroundedAction("Move", { "C", "Table", "A" }));
    // plan.push_back(GroundedAction("Move", { "B", "Table", "C" }));

    auto endTime = chrono::high_resolution_clock::now();

    auto duration = chrono::duration_cast<chrono::milliseconds>(endTime - startTime);
    cout << "Planning time: " << duration.count() << " ms" << endl;

    return plan;
}


int main(int argc, char* argv[])
{
    // DO NOT CHANGE THIS FUNCTION
    char* env_file = static_cast<char *>("example.txt");
    if (argc > 1)
        env_file = argv[1];
    std::string envsDirPath = ENVS_DIR;
    char* filename = new char[envsDirPath.length() + strlen(env_file) + 2];
    strcpy(filename, envsDirPath.c_str());
    strcat(filename, "/");
    strcat(filename, env_file);

    cout << "Environment: " << filename << endl;
    Env* env = create_env(filename);
    if (print_status)
    {
        cout << *env;
    }

    list<GroundedAction> actions = planner(env);

    cout << "\nPlan: " << endl;
    for (const GroundedAction& gac : actions)
    {
        cout << gac << endl;
    }

    delete env;
    return 0;
}