#include "parsed_parameter.h"
#include <sstream>

CCL_NAMESPACE_BEGIN

   void
   Parsed_Parameter::add_float( float v )
   {
      assert( strings.empty() && bools.empty() );
      floats.push_back( v );
   }

   void
   Parsed_Parameter::add_int( int i )
   {
      assert( strings.empty() && bools.empty() );
      ints.push_back( i );
   }

   void
   Parsed_Parameter::add_string( std::string_view str )
   {
      assert( floats.empty() && ints.empty() && bools.empty() );
      strings.push_back( {str.begin(), str.end()} );
   }

   void
   Parsed_Parameter::add_bool( bool v )
   {
      assert( floats.empty() && ints.empty() && strings.empty() );
      bools.push_back( v );
   }

   std::string
   Parsed_Parameter::to_string() const
   {
      std::stringstream ss;
      ss <<  "\"" << type << " " << name << "\" [ ";
      if ( !floats.empty() )
         for ( float d : floats ) ss << d << " ";
      else if ( !ints.empty() )
         for ( int i : ints ) ss << i << " ";
      else if ( !strings.empty() )
         for ( const auto &s : strings ) ss << "\"" << s << "\" ";
      else if ( !bools.empty() )
         for ( bool b : bools ) ss << (b ? "true " : "false ");
      ss << "] ";

      return ss.str();
   }

CCL_NAMESPACE_END
