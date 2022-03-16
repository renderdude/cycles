#ifndef PARSER_H
#define PARSER_H

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include "error.h"
#include "parsed_parameter.h"

CCL_NAMESPACE_BEGIN
    using Point3f = float[3];

   // Parser Definition
   class Parser
   {
    public:
      ~Parser();
      // Parser Interface
      void AreaLightSource( const std::string& name,
                                    Parsed_Parameter_Vector params,
                                    File_Loc loc );
      void Attribute( const std::string& target,
                              Parsed_Parameter_Vector params, File_Loc loc );
      void AttributeBegin( File_Loc loc );
      void AttributeEnd( File_Loc loc );
      void Camera( const std::string&, Parsed_Parameter_Vector params,
                           File_Loc loc );
      void ConcatTransform( float transform[ 16 ], File_Loc loc );
      void CoordinateSystem( const std::string&, File_Loc loc );
      void CoordSysTransform( const std::string&, File_Loc loc );
      void Identity( File_Loc loc );
      void Integrator( const std::string& name,
                               Parsed_Parameter_Vector params, File_Loc loc );
      void LightSource( const std::string& name,
                                Parsed_Parameter_Vector params, File_Loc loc );
      void ObjectBegin( const std::string& name, File_Loc loc );
      void ObjectEnd( File_Loc loc );
      void ObjectInstance( const std::string& name, File_Loc loc );
      void Option( const std::string& name, Parsed_Parameter_Vector params,
                           File_Loc loc );
      void PixelFilter( const std::string& name,
                                Parsed_Parameter_Vector params, File_Loc loc );
      void ReverseOrientation( File_Loc loc );
      void Rotate( float angle, float ax, float ay, float az, File_Loc loc );
      void Scale( float sx, float sy, float sz, File_Loc loc );
      void Texture( const std::string& name, const std::string& type,
                            const std::string& texname,
                            Parsed_Parameter_Vector params, File_Loc loc );
      void Transform( float transform[ 16 ], File_Loc loc );
      void Translate( float dx, float dy, float dz, File_Loc loc );
      void WorldBegin( File_Loc loc );

      void end_of_files();

      /*****************************************************************
       ****            Begin RenderMan Definitions                  ****
       *****************************************************************/
      void ArchiveBegin( const std::string& name,
                                 Parsed_Parameter_Vector params, File_Loc loc );
      void ArchiveEnd( File_Loc loc );
      void Atmosphere( const std::string& name,
                               Parsed_Parameter_Vector params, File_Loc loc );
      void Basis( Parsed_Parameter_Vector params, File_Loc loc );
      void Begin( const std::string& name, File_Loc loc );
      void Bound( float bound[ 6 ], File_Loc loc );
      void Bxdf( const std::string& bxdf, const std::string& name,
                         Parsed_Parameter_Vector params, File_Loc loc );
      void Clipping( float cnear, float cfar, File_Loc loc );
      void ClippingPlane( float x, float y, float z, float nx, float ny, float nz,
                                  File_Loc loc );
      void Color( float r, float g, float b, File_Loc loc );
      void Cone( float height, float radius, float thetamax,
                         Parsed_Parameter_Vector params, File_Loc loc );
      void CropWindow( float xmin, float xmax, float ymin, float ymax,
                               File_Loc loc );
      void Curves( const std::string& type, std::vector< int > nvertices,
                           const std::string& wrap, Parsed_Parameter_Vector params,
                           File_Loc loc );
      void Cylinder( float radius, float zmin, float zmax, float thetamax,
                             Parsed_Parameter_Vector params, File_Loc loc );
      void Declare( const std::string& name, const std::string& declaration,
                            File_Loc loc );
      void DepthOfField( float fstop, float focallength, float focaldistance,
                                 File_Loc loc );
      void Detail( float bound[ 6 ], File_Loc loc );
      void DetailRange( float offlow, float onlow, float onhigh, float offhigh,
                                File_Loc loc );
      void Disk( float height, float radius, float thetamax,
                         Parsed_Parameter_Vector params, File_Loc loc );
      void Displacement( const std::string& displace,
                                 const std::string& name,
                                 Parsed_Parameter_Vector params, File_Loc loc );
      void Display( const std::string& name, const std::string& type,
                            const std::string& mode, Parsed_Parameter_Vector params,
                            File_Loc loc );
      void DisplayChannel( const std::string& name,
                                   Parsed_Parameter_Vector params, File_Loc loc );
      void Else( File_Loc loc );
      void ElseIf( const std::string& condition, File_Loc loc );
      void End( File_Loc loc );
      void Exposure( float gain, float gamma, File_Loc loc );
      void Exterior( const std::string& name, Parsed_Parameter_Vector params,
                             File_Loc loc );
      void Format( int xresolution, int yresolution, float pixelaspectratio,
                           File_Loc loc );
      void FrameAspectRatio( float frameratio, File_Loc loc );
      void FrameBegin( int number, File_Loc loc );
      void FrameEnd( File_Loc loc );
      void GeneralPolygon( std::vector< int > nvertices,
                                   Parsed_Parameter_Vector params, File_Loc loc );
      void GeometricApproximation( const std::string& type, float value,
                                           File_Loc loc );
      void Geometry( const std::string& type, Parsed_Parameter_Vector params,
                             File_Loc loc );
      void Hider( const std::string& name, Parsed_Parameter_Vector params,
                          File_Loc loc );
      void Hyperboloid( Point3f point1, Point3f point2, float thetamax,
                                Parsed_Parameter_Vector params, File_Loc loc );
      void IfBegin( const std::string& condition, File_Loc loc );
      void IfEnd( File_Loc loc );
      void Illuminate( const std::string& light, bool onoff, File_Loc loc );
      void Imager( const std::string& name, Parsed_Parameter_Vector params,
                           File_Loc loc );
      void Integrator_RIB( const std::string& type, const std::string& name,
                                   Parsed_Parameter_Vector params, File_Loc loc );
      void Interior( const std::string& name, Parsed_Parameter_Vector params,
                             File_Loc loc );
      void Light_RIB( const std::string& name, const std::string& handle,
                              Parsed_Parameter_Vector params, File_Loc loc );
      void MakeBrickMap( std::vector< std::string > ptcnames,
                                 const std::string& bkmname,
                                 Parsed_Parameter_Vector params, File_Loc loc );
      void MakeCubeFaceEnvironment(
          const std::string& px, const std::string& nx, const std::string& py,
          const std::string& ny, const std::string& pz, const std::string& nz,
          const std::string& text, float fov, const std::string& filt, float swidth,
          float twidth, Parsed_Parameter_Vector params, File_Loc loc );
      void MakeLatLongEnvironment( const std::string& picturename,
                                           const std::string& texturename,
                                           const std::string& filt, float swidth,
                                           float twidth,
                                           Parsed_Parameter_Vector params,
                                           File_Loc loc );
      void MakeShadow( const std::string& picturename,
                               const std::string& texturename,
                               Parsed_Parameter_Vector params, File_Loc loc );
      void MakeTexture( const std::string& picturename,
                                const std::string& texturename,
                                const std::string& swrap, const std::string& twrap,
                                const std::string& filt, float swidth, float twidth,
                                Parsed_Parameter_Vector params, File_Loc loc );
      void Matte( bool onoff, File_Loc loc );
      void MotionBegin( std::vector< float > times, File_Loc loc );
      void MotionEnd( File_Loc loc );
      void NuPatch( int nu, int uorder, float uknot[], float umin, float umax,
                            int nv, int vorder, float vknot[], float vmin, float vmax,
                            Parsed_Parameter_Vector params, File_Loc loc );
      void Opacity( Parsed_Parameter_Vector params, File_Loc loc );
      void Orientation( const std::string& orientation, File_Loc loc );
      void Paraboloid( float rmax, float zmin, float zmax, float thetamax,
                               Parsed_Parameter_Vector params, File_Loc loc );
      void Patch( const std::string& type, Parsed_Parameter_Vector params,
                          File_Loc loc );
      void PatchMesh( const std::string& type, int nu,
                              const std::string& uwrap, int nv,
                              const std::string& vwrap,
                              Parsed_Parameter_Vector params, File_Loc loc );
      void Pattern( const std::string& name, const std::string& handle,
                            Parsed_Parameter_Vector params, File_Loc loc );
      void Perspective( float fov, File_Loc loc );
      void PixelSampleImager( const std::string& name,
                                      Parsed_Parameter_Vector params, File_Loc loc );
      void PixelSamples( float xsamples, float ysamples, File_Loc loc );
      void PixelVariance( float variance, File_Loc loc );
      void Points( int npoints, Parsed_Parameter_Vector params,
                           File_Loc loc );
      void PointsGeneralPolygons( std::vector< int > n_loops,
                                          std::vector< int > n_vertices,
                                          std::vector< int > vertices,
                                          Parsed_Parameter_Vector params,
                                          File_Loc loc );
      void PointsPolygons( std::vector< int > n_vertices,
                                   std::vector< int > vertices,
                                   Parsed_Parameter_Vector params, File_Loc loc );
      void Polygon( int nvertices, Parsed_Parameter_Vector params,
                            File_Loc loc );
      void ProcDelayedReadArchive( Parsed_Parameter_Vector params,
                                           File_Loc loc );
      void ProcDynamicLoad( Parsed_Parameter_Vector params, File_Loc loc );
      void Procedural( const std::string& proc_name,
                               Parsed_Parameter_Vector params, File_Loc loc );
      void Procedural2( const std::string& proc_name,
                                Parsed_Parameter_Vector params, File_Loc loc );
      void ProcFree( Parsed_Parameter_Vector params, File_Loc loc );
      void ProcRunProgram( Parsed_Parameter_Vector params, File_Loc loc );
      void Projection( const std::string& name,
                               Parsed_Parameter_Vector params, File_Loc loc );
      void Quantize( const std::string& type, int one, int min, int max,
                             float ditheramplitude, File_Loc loc );
      void ReadArchive( const std::string& name,
                                Parsed_Parameter_Vector params, File_Loc loc );
      void RelativeDetail( float relativedetail, File_Loc loc );
      void Resource( const std::string& handle, const std::string& type,
                             Parsed_Parameter_Vector params, File_Loc loc );
      void ResourceBegin( File_Loc loc );
      void ResourceEnd( File_Loc loc );
      void ScopedCoordinateSystem( const std::string&, File_Loc loc );
      void ScreenWindow( float left, float right, float bottom, float top,
                                 File_Loc loc );
      void ShadingInterpolation( const std::string& type, File_Loc loc );
      void ShadingRate( float size, File_Loc loc );
      void Shutter( float opentime, float closetime, File_Loc loc );
      void Sides( int nsides, File_Loc loc );
      void Skew( float angle, float dx1, float dy1, float dz1, float dx2,
                         float dy2, float dz2, File_Loc loc );
      void SolidBegin( const std::string& type, File_Loc loc );
      void SolidEnd( File_Loc loc );
      void Sphere( float radius, float zmin, float zmax, float thetamax,
                           Parsed_Parameter_Vector params, File_Loc loc );
      void SubdivisionMesh( const std::string& scheme, int nfaces,
                                    Parsed_Parameter_Vector params, File_Loc loc );
      void Surface( const std::string& name, Parsed_Parameter_Vector params,
                            File_Loc loc );
      void System( const std::string& cmd, File_Loc loc );
      void TextureCoordinates( float s1, float t1, float s2, float t2, float s3,
                                       float t3, float s4, float t4, File_Loc loc );
      void Torus( float majorrad, float minorrad, float phimin, float phimax,
                          float thetamax, Parsed_Parameter_Vector params,
                          File_Loc loc );
      void TransformBegin( File_Loc loc );
      void TransformEnd( File_Loc loc );
      void TrimCurve( int nloops, int ncurves[], int order[], float knot[],
                              float min[], float max[], int n[], float u[], float v[],
                              float w[], File_Loc loc );
      void WorldEnd( File_Loc loc );

    protected:
      // Parser Protected Methods
      template < typename... Args >
      void
      error_exit_deferred( const char* fmt, Args&&... args ) const
      {
         errorExit = true;
         error( fmt, std::forward< Args >( args )... );
      }
      template < typename... Args >
      void
      error_exit_deferred( const File_Loc* loc, const char* fmt,
                           Args&&... args ) const
      {
         errorExit = true;
         error( loc, fmt, std::forward< Args >( args )... );
      }

      mutable bool errorExit = false;
   };

   // Scene Parsing Declarations
   void parse_files( Parser* target,
                     std::vector< std::string > filenames );
   void parse_string( Parser* target, std::string str );

   // Token Definition
   struct Token
   {
      Token() = default;
      Token( std::string_view token, File_Loc loc ): token( token ), loc( loc ) {}
      std::string to_string() const;
      std::string_view token;
      File_Loc loc;
   };

   // Tokenizer Definition
   class Tokenizer
   {
    public:
      // Tokenizer Public Methods
      Tokenizer(
          std::string str, std::string filename,
          std::function< void( const char*, const File_Loc* ) > error_callback );
#if defined( HAVE_MMAP ) || defined( IS_WINDOWS )
      Tokenizer(
          void* ptr, size_t len, std::string filename,
          std::function< void( const char*, const File_Loc* ) > error_callback );
#endif
      ~Tokenizer();

      static std::unique_ptr< Tokenizer > create_from_file(
          const std::string& filename,
          std::function< void( const char*, const File_Loc* ) > error_callback );
      static std::unique_ptr< Tokenizer > create_from_string(
          std::string str,
          std::function< void( const char*, const File_Loc* ) > error_callback );

      std::optional< Token > Next();

      // Just for parse().
      // TODO? Have a method to set this?
      File_Loc loc;

    private:
      void check_utf( const void* ptr, int len ) const;
      // Tokenizer Private Methods
      int
      get_char()
      {
         if ( pos == end ) return EOF;
         int ch = *pos++;
         if ( ch == '\n' )
         {
            ++loc.line;
            loc.column = 0;
         }
         else
            ++loc.column;
         return ch;
      }
      void
      unget_char()
      {
         --pos;
         if ( *pos == '\n' )
            // Don't worry about the column; we'll be going to the start of
            // the next line again shortly...
            --loc.line;
      }

      // Tokenizer Private Members
      // This function is called if there is an error during lexing.
      std::function< void( const char*, const File_Loc* ) > error_callback;

#if defined( HAVE_MMAP ) || defined( IS_WINDOWS )
      // Scene files on disk are mapped into memory for lexing.  We need to
      // hold on to the starting pointer and total length so they can be
      // unmapped in the destructor.
      void* unmapPtr     = nullptr;
      size_t unmapLength = 0;
#endif

      // If the input is stdin, then we copy everything until EOF into this
      // string and then start lexing.  This is a little wasteful (versus
      // tokenizing directly from stdin), but makes the implementation
      // simpler.
      std::string contents;

      // Pointers to the current position in the file and one past the end of
      // the file.
      const char *pos, *end;

      // If there are escaped characters in the string, we can't just return
      // a std::string_view into the mapped file. In that case, we handle the
      // escaped characters and return a std::string_view to sEscaped.  (And
      // thence, std::string_views from previous calls to Next() must be invalid
      // after a subsequent call, since we may reuse sEscaped.)
      std::string sEscaped;
   };

CCL_NAMESPACE_END

#endif  // PARSER_H
