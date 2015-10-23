//===========================================================================
// Copied from boost escaped_list_seperator

#include <boost/tokenizer.hpp>

struct escaped_forth_error : public std::runtime_error{
escaped_forth_error(const std::string& what_arg):std::runtime_error(what_arg) { }
};

template <class Char,
	  class Traits = BOOST_DEDUCED_TYPENAME std::basic_string<Char>::traits_type >
class escaped_forth_separator {

private:
  typedef std::basic_string<Char,Traits> string_type;
  struct char_eq {
    Char e_;
    char_eq(Char e):e_(e) { }
    bool operator()(Char c) {
      return Traits::eq(e_,c);
    }
  };
  string_type  escape_;
  string_type  c_;
  string_type  quote_;
  bool last_;

  bool is_escape(Char e) {
    char_eq f(e);
    return std::find_if(escape_.begin(),escape_.end(),f)!=escape_.end();
  }
  bool is_c(Char e) {
    char_eq f(e);
    return std::find_if(c_.begin(),c_.end(),f)!=c_.end();
  }
  bool is_quote(Char e) {
    char_eq f(e);
    return std::find_if(quote_.begin(),quote_.end(),f)!=quote_.end();
  }

  template <typename iterator, typename Token>
  void do_escape(iterator& next,iterator end,Token& tok) {
    if (++next == end)
      throw escaped_forth_error(std::string("cannot end with escape"));
    if (Traits::eq(*next,'n')) {
      tok+='\n';
      return;
    }
    else if (is_quote(*next)) {
      tok+=*next;
      return;
    }
    else if (is_c(*next)) {
      tok+=*next;
      return;
    }
    else if (is_escape(*next)) {
      tok+=*next;
      return;
    }
    else
      throw escaped_forth_error(std::string("unknown escape sequence"));
  }

public:

  explicit escaped_forth_separator(Char  e = '\\',
				  Char c = ',',Char  q = '\"')
    : escape_(1,e), c_(1,c), quote_(1,q), last_(false) { }

  escaped_forth_separator(string_type e, string_type c, string_type q)
    : escape_(e), c_(c), quote_(q), last_(false) { }

  void reset() {last_=false;}

  template <typename InputIterator, typename Token>
  bool operator()(InputIterator& next,InputIterator end,Token& tok) {
    bool bInQuote = false;
    bool bInComment = false;
    tok = Token();

    if (next == end) {
      if (last_) {
	last_ = false;
	return true;
      }
      else
	return false;
    }
    last_ = false;
    for (;next != end;++next) {
      if (is_escape(*next)) {
	do_escape(next,end,tok);
      }
      else if (is_c(*next)) {
	if (!bInQuote && !bInComment) {
	  // If we are not in quote, then we are done
	  ++next;
	  // The last character was a c, that means there is
	  // 1 more blank field
	  last_ = true;
	  return true;
	}
	else tok+=*next;
      }
      else if (*next == '/' && !bInQuote) {
	tok += *next;
	if (++next == end) {
	  return true;
	}
	if (*next == '*') {
	  bInComment = true;
	}
	tok += *next;
      }
      else if (*next == '*' && !bInQuote) {
	tok += *next;
	if (++next == end) {
	  return true;
	}

	if (*next == '/') {
	  bInComment = false;
	} else {
	  next--;
	}
      }
      else if (is_quote(*next) && !bInComment) {
	bInQuote=!bInQuote;
	tok += *next;
      }
      else {
	tok += *next;
      }
    }
    return true;
  }
};
