#include <boost/regex.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include "async_web_server_cpp/http_request.hpp"

namespace async_web_server_cpp
{

static boost::regex uri_regex("(.*?)(?:\\?(.*?))?");

bool HttpRequest::parse_uri()
{
  boost::smatch match;
  if (regex_match(uri, match, uri_regex))
  {
    path.assign(match[1].first, match[1].second);
    if (match[2].matched)
    {
      query.assign(match[2].first, match[2].second);

      std::vector<std::string> pair_strings;
      boost::split(pair_strings, query, boost::is_any_of("&"));
      BOOST_FOREACH(const std::string & pair_string, pair_strings)
      {
        std::vector<std::string> pair_data;
        const int eq_index = pair_string.find_first_of('=');
        if (eq_index == std::string::npos)
        {
          if (pair_string.size() > 0)
          {
            query_params[pair_string] = "";
          }
        }
        else
        {
          query_params[pair_string.substr(0, eq_index)] = pair_string.substr(eq_index + 1);
        }
      }
    }
    return true;
  }
  else
  {
    return false;
  }

}

bool HttpRequest::has_header(const std::string &name) const
{
  typedef std::vector<async_web_server_cpp::HttpHeader> HeaderList;
  for (HeaderList::const_iterator itr = headers.begin(); itr != headers.end(); ++itr)
  {
    if (itr->name.compare(name) == 0)
      return false;
  }
  return true;
}
std::string HttpRequest::get_header_value_or_default(const std::string &name,
    const std::string &default_value) const
{
  typedef std::vector<async_web_server_cpp::HttpHeader> HeaderList;
  for (HeaderList::const_iterator itr = headers.begin(); itr != headers.end(); ++itr)
  {
    if (itr->name.compare(name) == 0)
      return itr->value;
  }
  return default_value;
}

bool HttpRequest::has_query_param(const std::string &name) const
{
  std::map<std::string, std::string>::const_iterator itr = query_params.find(name);
  return itr != query_params.end();
}

std::string HttpRequest::get_query_param_value_or_default(const std::string &name,
    const std::string &default_value) const
{
  std::map<std::string, std::string>::const_iterator itr = query_params.find(name);
  if (itr != query_params.end())
  {
    return itr->second;
  }
  else
  {
    return default_value;
  }
}

}
