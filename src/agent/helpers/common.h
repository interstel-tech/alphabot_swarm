#include "support/convertlib.h"
#include <string>

struct NodeState
{
    std::string nodename;
    Convert::cartpos lvlh;
    json11::Json to_json() const {
        return json11::Json::object {
            { "nodename" , nodename },
            { "lvlh", lvlh },
        };
    }
    void from_json(const string& s)	{
        string error;
        json11::Json parsed = json11::Json::parse(s,error);
        if(error.empty())	{
            if(!parsed["nodename"].is_null()) { nodename = parsed["nodename"].string_value(); }
            if(!parsed["lvlh"].is_null()) { lvlh.from_json(parsed["lvlh"].dump()); }
        } else {
            cerr<<"ERROR: <"<<error<<">"<<endl;
        }
    }
};
