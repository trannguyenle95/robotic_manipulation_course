
#include <iostream>
#include <list>
#include <fstream>
#include <algorithm>
#include <direct.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/assign/std/vector.hpp>
#include <boost/random/random_device.hpp>
#include <boost/random/uniform_int_distribution.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/units/unit.hpp>
#include <boost/units/base_units/angle/radian.hpp>
#include <boost/units/base_units/angle/degree.hpp>
#include <boost/units/conversion.hpp>
#include <boost/units/static_constant.hpp>
#include <boost/units/make_system.hpp>
#include <boost/units/quantity.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/operation.hpp>
#include <boost/geometry/algorithms/covered_by.hpp>
#include <boost/filesystem.hpp>


#define X_COORD (0)
#define Y_COORD (1)
#define QUAD_1  (1)
#define QUAD_2  (2)
#define QUAD_3  (3)
#define QUAD_4  (4)

#define STEP_COORD         (0.0001)
#define MARGIN             (0.5)
#define HALF_MARGIN        (0.25)
#define DELTA_SENSITIVITY  (100)

#define MU_FRICTION_ANGLE  (45)

#define VERSOR_LENGTH      (2.5)

#define MUJOCO_SCALE_FACTOR   (0.1)


#define EX_ERROR        (-1)
#define EX_OK           (0)
#define EX_NO_INTERSEC  (-2)

#define GET_X(point)      (bg::get<X_COORD>(point))
#define GET_Y(point)      (bg::get<Y_COORD>(point))

#define SET_X(point, x)   (bg::set<X_COORD>(point, x))
#define SET_Y(point, y)   (bg::set<Y_COORD>(point, y))

#define POINT_MIN_X(p1, p2)  (( GET_X(p1) < GET_X(p2) ) ? p1 : p2)

#define POINT_MAX_X(p1, p2)  (( GET_X(p1) > GET_X(p2) ) ? p1 : p2)

#define POINT_MIN_Y(p1, p2)  (( GET_Y(p1) < GET_Y(p2) ) ? p1 : p2)

#define POINT_MAX_Y(p1, p2)  (( GET_Y(p1) > GET_Y(p2) ) ? p1 : p2)


namespace bg = boost::geometry;


typedef bg::model::d2::point_xy<double> Point2d_t;
typedef bg::model::polygon<Point2d_t> Polygon2d_t;
typedef bg::model::segment<Point2d_t> Segment2d_t;
typedef bg::model::linestring<Point2d_t> LineString2d_t;


Point2d_t _polyMinPointX(0, 0), _polyMinPointY(0, 0), _polyMaxPointX(0, 0), _polyMaxPointY(0, 0);

Point2d_t _versDir(0, 0), _versFup(0, 0), _versFdown(0, 0);

Polygon2d_t _poly;

LineString2d_t _firstLineDir, _firstConeUp, _firstConeDown;

LineString2d_t _secondLineDir, _secondConeUp, _secondConeDown;

Point2d_t _firstContactPoint(0,0), _secondContactPoint(0,0), _centroid(0,0);

std::vector<Point2d_t> _polygonPoints;

double _firstContactAngle, _secondContactAngle;

std::string _dirPath, _resultPath, _imagePath, _imagePathNoSolution;

bool _optimal = false;

Point2d_t _polyV1, _polyV2;


int readPolyfromFile(char * pathIn);

void plot(bool plotVersor, bool plotContour, std::string & path);

void plotContactCone();

void getPolyExtremeVertex(std::vector<Point2d_t> & readVertex);

int getPointQuadrant(Point2d_t & point);

bool getIntersectPoint(Polygon2d_t & _poly, LineString2d_t & coneLine, Point2d_t & intersectPoint, Point2d_t & contactPoint);

double toRadians(const double angleInDegrees);

double toDegrees(double angleInRadians);

void createTargetPoint(double angle, Point2d_t & fromPoint, Point2d_t & targetPoint);

void createVersorPoints(Point2d_t & direction, Point2d_t & upFriction, Point2d_t & downFriction, double angle);

void rotatePoints(Point2d_t & direction, Point2d_t & upFriction, Point2d_t & downFriction, double angle);

void rotateVersorPoint(Point2d_t & point, double angle);

void translatePoints(Point2d_t & direction, Point2d_t & upFriction, Point2d_t & downFriction, Point2d_t & _firstContactPoint);

void plotContour(std::string & path);

double getQ(Point2d_t & p1, Point2d_t & p2);

double getM(Point2d_t & p1, Point2d_t & p2);

void getPolyEdgesPoints(std::vector<Point2d_t> polyRingArray);

void findPointBetween(std::vector<Point2d_t> & intersectPoints, Point2d_t & directionPoint, Point2d_t & contactPoint, Point2d_t & outPoint);

bool isBetweenPoint(Point2d_t & toCheck, std::vector<Point2d_t> & intersectPoints, Point2d_t & directionPoint);

void getSecondPointContactAngle();

void createPath(char * folderOut);

void writeOutput();

void getAllPointsBetween(Point2d_t & a, Point2d_t & b, std::vector<Point2d_t> & points);

const double p_greek = boost::math::constants::pi<double>();

void get2PolyVertex(std::vector<Point2d_t> vertices)
{
    boost::random::random_device rng;

    boost::random::uniform_int_distribution<> dist(0, vertices.size()-1);

    int index1 = dist(rng);
    int index2 = dist(rng);

    while( index1 == index2 )
    {
        index2 = dist(rng);
    }

    SET_X(_polyV1, GET_X(vertices[ index1 ])); 
    SET_Y(_polyV1, GET_Y(vertices[ index1 ]));

    SET_X(_polyV2, GET_X(vertices[ index2 ]));
    SET_Y(_polyV2, GET_Y(vertices[ index2 ]));

}//[f]


int getPolyRandomContactPoint()
{
    std::vector<Point2d_t> pointBetween;

    getAllPointsBetween(_polyV1, _polyV2, pointBetween);

    boost::random::random_device rng;

    if( pointBetween.size() < 2 )
    {
        return EX_ERROR;
    }

    boost::random::uniform_int_distribution<> dist(0, pointBetween.size() - 1);

    int index = dist(rng);

    SET_X(_firstContactPoint, GET_X(pointBetween[index]) );

    SET_Y(_firstContactPoint, GET_Y(pointBetween[ index ]));

    return EX_OK;

}//[f]

void getFirstAngle()
{
    int quadrant = getPointQuadrant(_firstContactPoint);

    double angle = 0;

    double m = getM(_polyV1, _polyV2);

    if( std::abs(m) < 0.00001 )
    {
        angle = 90;

        if( ( QUAD_1 == quadrant ) || ( QUAD_2 == quadrant ) )
        {
            _firstContactAngle = angle + 180;
        }
        else
        {
            _firstContactAngle = angle;
        }
    }
    else
    {
        angle = toDegrees(std::atan(-m));

        if( ( QUAD_1 == quadrant ) || ( QUAD_4 == quadrant ) )
        {
            _firstContactAngle = angle + 180;
        }
        else
        {
            _firstContactAngle = angle;
        }
    }
      

}//[f]


int main(int argc, char * argv[])
{
    if( argc < 3 )
    {
        std::cout << "launch program: name_exe path_data_file_in path_dir_out" << std::endl;
        return EX_ERROR;
    }

    createPath(argv[2]);

    LineString2d_t versLineDir, versLineUp, versLineDown;
    
    if( EX_ERROR == readPolyfromFile(argv[1]) )
    {
        std::cout << "Error on reading vertices file" << std::endl;
        return EX_ERROR;
    }
       
    bg::centroid(_poly, _centroid);

    std::cout << "centroid: " << bg::dsv(_centroid) << std::endl;
 
    int error = getPolyRandomContactPoint(); 

    if( error == EX_ERROR )
    {
        std::cout << "error on getting contact point" << std::endl;

        plotContour(_imagePathNoSolution);

        return EX_ERROR;
    }

    getFirstAngle();
    
    //create contact Line
    createVersorPoints(_versDir, _versFup, _versFdown, MU_FRICTION_ANGLE);  

    Point2d_t firstDir, firstUp, firstDown;

    firstDir = _versDir;
    firstUp = _versFup;
    firstDown = _versFdown;

    //apply direction angle
    rotatePoints(firstDir, firstUp, firstDown, _firstContactAngle);

    //translate to final position
    translatePoints(firstDir, firstUp, firstDown, _firstContactPoint);

    
    //create cones line string
    bg::append(_firstLineDir, _firstContactPoint);
    bg::append(_firstLineDir, firstDir);

    bg::append(_firstConeUp, _firstContactPoint);
    bg::append(_firstConeUp, firstUp);

    bg::append(_firstConeDown, _firstContactPoint);
    bg::append(_firstConeDown, firstDown);

    std::cout << "contact point: " << bg::dsv(_firstContactPoint) << std::endl
        << "contact angle: " << _firstContactAngle << std::endl
        << "angle PLUS: " << _firstContactAngle + MU_FRICTION_ANGLE << std::endl
        << "angle MINUS: " << _firstContactAngle - MU_FRICTION_ANGLE << std::endl
        << std::endl;

   
    Point2d_t firstIntersectUp(0, 0), firstIntersectDown(0, 0), firstIntersectDir(0, 0);

    bool intersec1 = getIntersectPoint(_poly, _firstConeUp, firstIntersectUp, _firstContactPoint);

    bool intersec2 = getIntersectPoint(_poly, _firstConeDown, firstIntersectDown, _firstContactPoint);

    bool intersec3 = getIntersectPoint(_poly, _firstLineDir, firstIntersectDir, _firstContactPoint);

    std::cout << "Intersect first cone up is: " << intersec1 << " ..value is:" << bg::dsv(firstIntersectUp) << std::endl;

    std::cout << "Intersect first cone down is: " << intersec2 << " ..value is:" << bg::dsv(firstIntersectDown) << std::endl;

    std::cout << "Intersect first direction is: " << intersec3 << " ..value is:" << bg::dsv(firstIntersectDir) << std::endl;

    std::cout << std::endl;

    if( !intersec3 )
    {
        std::cout << "CANNOT find second contact point (1)! Exit program" << std::endl;

        plotContactCone();

        return EX_ERROR;
    }

    SET_X(_secondContactPoint, GET_X( firstIntersectDir));
    SET_Y(_secondContactPoint, GET_Y(firstIntersectDir));

    //get quadrant of the second contact point
    int quadrant = getPointQuadrant(_secondContactPoint);
   
    getSecondPointContactAngle();

    std::cout << "second contact angle: " << _secondContactAngle << std::endl;
       

    Point2d_t secondDir, secondUp, secondDown;

    secondDir = _versDir;
    secondUp = _versFup;
    secondDown = _versFdown;

    //apply direction angle
    rotatePoints(secondDir, secondUp, secondDown, _secondContactAngle);

    //translate point
    translatePoints(secondDir, secondUp, secondDown, _secondContactPoint);

    _secondLineDir.clear();
    _secondConeUp.clear();
    _secondConeDown.clear();

    //create cones line string
    bg::append(_secondLineDir, _secondContactPoint);
    bg::append(_secondLineDir, secondDir);

    bg::append(_secondConeUp, _secondContactPoint);
    bg::append(_secondConeUp, secondUp);

    bg::append(_secondConeDown, _secondContactPoint);
    bg::append(_secondConeDown, secondDown);

    Point2d_t secondIntersectUp(0,0), secondIntersectDown(0,0), secondIntersectDir(0,0);

    intersec1 = getIntersectPoint(_poly, _secondConeUp, secondIntersectUp, _secondContactPoint);

    intersec2 = getIntersectPoint(_poly, _secondConeDown, secondIntersectDown, _secondContactPoint);

    intersec3 = getIntersectPoint(_poly, _secondLineDir, secondIntersectDir, _secondContactPoint);

    std::cout << "second contact point: " << bg::dsv(_secondContactPoint) << std::endl;

    std::cout << "Intersect second cone is:" << bg::dsv(secondIntersectUp) << std::endl;

    std::cout << "Intersect second cone down is:" << bg::dsv(secondIntersectDown) << std::endl;

    std::cout << "Intersect second direction is:" << bg::dsv(secondIntersectDir) << std::endl;

    std::cout << std::endl;    

   plot(false, false, _imagePath);
 

   writeOutput();
  
   return EX_OK;
}//[f]


double getM(Point2d_t & p1, Point2d_t & p2)
{
    double m;

    double den = GET_X(p1) - GET_X(p2);
    double num = GET_Y(p1) - GET_Y(p2);

    m = num / den;

    return m;
}//[f]

double getQ(Point2d_t & p1, Point2d_t & p2)
{
    double q;

    double denum = GET_X(p1) - GET_X(p2);

    double num = GET_X(p1)*GET_Y(p2) - GET_X(p2)*GET_Y(p1);

    q = num / denum;

    return q;
}//[f]

void getAllPointsBetween(Point2d_t & a, Point2d_t & b, std::vector<Point2d_t> & points)
{
    double minX, maxX, minY, maxY;

    minX = GET_X(POINT_MIN_X(a, b));

    maxX = GET_X(POINT_MAX_X(a, b));

    minY = GET_Y(POINT_MIN_Y(a, b));

    maxY = GET_Y(POINT_MAX_Y(a, b));

    for( unsigned i = 0; i < _polygonPoints.size(); i++ )
    {
        //std::cout << bg::dsv(_polygonPoints[ i ]) << std::endl;

        if( ( GET_X(_polygonPoints[ i ]) >= minX ) && ( GET_X(_polygonPoints[ i ]) <= maxX ) )
        {
            if( ( GET_Y(_polygonPoints[ i ]) >= minY ) && ( GET_Y(_polygonPoints[ i ]) <= maxY ) )
            {
                points.push_back(_polygonPoints[ i ]);
            }
        }
    }


}//[f]

bool isBetweenPoint(Point2d_t & toCheck, std::vector<Point2d_t> & intersectPoints, Point2d_t & directionPoint)
{
    int dim = intersectPoints.size();

    bool isInside = false;

    if( dim == 0 )
    {
        return isInside;
    }
        

    std::vector<Point2d_t> goodPoints;
    Point2d_t minPointX(0, 0), maxPointX(0, 0);

    if( dim > 1 )
    {
        minPointX = ( GET_X(intersectPoints[ 0 ]) < GET_X(intersectPoints[ 1 ]) ) ? intersectPoints[ 0 ] : intersectPoints[ 1 ];

        maxPointX = ( GET_X(intersectPoints[ 0 ]) > GET_X(intersectPoints[ 1 ]) ) ? intersectPoints[ 0 ] : intersectPoints[ 1 ];
    }
    else
    {
        minPointX = ( GET_X(intersectPoints[ 0 ]) < GET_X(directionPoint) ) ? intersectPoints[ 0 ] : directionPoint;

        maxPointX = ( GET_X(intersectPoints[ 0 ]) > GET_X(directionPoint) ) ? intersectPoints[ 0 ] : directionPoint;
    }

    for( unsigned i = 0; i < _polygonPoints.size(); i++ )
    {
        if( ( GET_X(_polygonPoints[ i ]) > GET_X(minPointX) ) && ( GET_X(_polygonPoints[ i ]) < GET_X(maxPointX) ) )
        {
            goodPoints.push_back(_polygonPoints[ i ]);
        }
    }

    if( goodPoints.size() > 0 )
    {
        for( unsigned i = 0; i < _polygonPoints.size(); i++ )
        {
            double dist = boost::geometry::distance(_polygonPoints[ i ], toCheck);

            //skip false positive due to aproximation error
            if( dist < 0.000001 )
            {
                isInside = true;
            }
        }
    }
    else
    {
        isInside = false;
    }

    return isInside;
    
}//[f]

void getSecondPointContactAngle()
{
    _secondContactAngle = _firstContactAngle - 180;

}//[f]


void findPointBetween(std::vector<Point2d_t> & intersectPoints, Point2d_t & directionPoint, Point2d_t & contactPoint, Point2d_t & outPoint)
{
    boost::random::random_device rng;

    int dim = intersectPoints.size();

    if( dim == 0 )
        return;

    std::vector<Point2d_t> goodPoints;
    Point2d_t minPointX(0, 0), maxPointX(0, 0);

    if( dim > 2 )
    {
        getAllPointsBetween(intersectPoints[ 0 ], intersectPoints[ 2 ], goodPoints);
        getAllPointsBetween(intersectPoints[ 2 ], intersectPoints[ 1 ], goodPoints);
    }
    else
    {
        getAllPointsBetween(intersectPoints[ 0 ], contactPoint, goodPoints);

       // getAllPointsBetween(directionPoint, contactPoint, goodPoints);
    }


    if( goodPoints.size() > 0 )
    {   //get a point with a good distance from the two intersection points
        int a = (int) std::round(goodPoints.size() * 0.3333);
        int b = (int) std::round(goodPoints.size() * 0.6666);

        boost::random::uniform_int_distribution<> dist(a, b);

        outPoint = goodPoints[ ((dist(rng) < (int)goodPoints.size())? dist(rng) : 0) ];
    }
    else
    {
        outPoint = Point2d_t(0, 0);
    }

    if( _optimal )
    {
        double minDist = std::numeric_limits<double>::max();
        double dist = 0;

        for( Point2d_t & p: goodPoints)
        {
            dist = bg::distance(_centroid, p);

            if( minDist > dist )
            {
                minDist = dist;
                outPoint = p;
            }                     
        }
    }



}//[f]


void getPolyEdgesPoints(std::vector<Point2d_t> polyRingArray)
{
    for( unsigned index = 0; index < polyRingArray.size() - 1; index++ )
    {
        double x1 = GET_X(polyRingArray[ index ]);
        double x2 = GET_X(polyRingArray[ index + 1 ]);

        double xMin = std::min(x1, x2);
        double xMax = std::max(x1, x2);

        double xdelta = std::abs(x1 - x2);

        double m = getM(polyRingArray[ index ], polyRingArray[ index + 1 ]);
        double q = getQ(polyRingArray[ index ], polyRingArray[ index + 1 ]);

        double y;

        double xStep = xdelta / DELTA_SENSITIVITY;

        _polygonPoints.push_back(polyRingArray[ index ]);

        for( double x = xMin + xStep; x < xMax; x += xStep )
        {
            y = m * x + q;

            _polygonPoints.push_back(Point2d_t(x, y));
        }
    }

    //last point pair

    double x1 = ( GET_X(polyRingArray[ polyRingArray.size() - 1 ]));
    double x2 = GET_X(polyRingArray[ 0 ]);

    double xMin = std::min(x1,x2);
    double xMax = std::max(x1, x2);

    double xdelta = std::abs(x1 - x2);

    double m = getM(polyRingArray[ polyRingArray.size() - 1 ], polyRingArray[ 0 ]);
    double q = getQ(polyRingArray[ polyRingArray.size() - 1 ], polyRingArray[ 0 ]);

    double y;

    double xStep = xdelta / DELTA_SENSITIVITY;

    _polygonPoints.push_back(polyRingArray[ polyRingArray.size() - 1 ]);

    for( double x = xMin + xStep; ( x < xMax ); x += xStep )
    {
        y = m * x + q;

        _polygonPoints.push_back(Point2d_t(x, y));
    }

    _polygonPoints.push_back(polyRingArray[0]);

}//[f]


void createTargetPoint(double angle, Point2d_t & fromPoint, Point2d_t & targetPoint)
{
    using namespace boost::numeric::ublas;

    matrix<double> M(2, 2);
    vector<double> initial(2);
    vector<double> target(2);

    double angRad = toRadians(angle);

    //row 0
    M(0, 0) = std::cos(angRad);
    M(0, 1) = ( -1 ) * std::sin(angRad);

    //row 1
    M(1, 0) = std::sin(angRad);
    M(1, 1) = std::cos(angRad);


    initial(0) = GET_X(fromPoint);
    initial(1) = GET_Y(fromPoint);

    axpy_prod(M, initial, target, true);

    SET_X(targetPoint, target(0));

    SET_Y(targetPoint, target(1));

    /*
    std::cout << initial << std::endl;

    std::cout << M << std::endl;

    std::cout << target << std::endl;
    */
}//[f]


void createVersorPoints(Point2d_t & direction, Point2d_t & upFriction, Point2d_t & downFriction, double angle)
{
    using namespace boost::numeric::ublas;

    SET_X(direction, VERSOR_LENGTH);

    SET_Y(direction, 0.0);

    createTargetPoint(angle, direction, upFriction);

    createTargetPoint((-angle), direction, downFriction);

}//[f]


void rotateVersorPoint(Point2d_t & point, double angle)
{
    using namespace boost::numeric::ublas;

    matrix<double> M(2, 2);
    vector<double> initial(2);
    vector<double> target(2);

    double angRad = toRadians(angle);

    //row 0
    M(0, 0) = std::cos(angRad);
    M(0, 1) = ( -1 ) * std::sin(angRad);

    //row 1
    M(1, 0) = std::sin(angRad);
    M(1, 1) = std::cos(angRad);

    initial(0) = GET_X(point);
    initial(1) = GET_Y(point);

    axpy_prod(M, initial, target, true);

    SET_X(point, target(0));

    SET_Y(point, target(1));

    /*
    std::cout << initial << std::endl;

    std::cout << M << std::endl;

    std::cout << target << std::endl;
    */
}//[f]


void translatePoint(Point2d_t & point, Point2d_t & target)
{
    double x = GET_X(point) + GET_X(target);

    double y = GET_Y(point) + GET_Y(target);

    SET_X(point, x);

    SET_Y(point, y);

}//[f]


void translatePoints(Point2d_t & direction, Point2d_t & upFriction, Point2d_t & downFriction, Point2d_t & _firstContactPoint)
{
    translatePoint(direction, _firstContactPoint);

    translatePoint(upFriction, _firstContactPoint);

    translatePoint(downFriction, _firstContactPoint);

}//[f]


void rotatePoints(Point2d_t & direction, Point2d_t & upFriction, Point2d_t & downFriction, double angle)
{
    rotateVersorPoint(direction, angle);

    rotateVersorPoint(upFriction, (angle));

    rotateVersorPoint(downFriction, (angle));

}//[f]


void getPolyExtremeVertex(std::vector<Point2d_t> & readVertex)
{
    double x, y;
    
    SET_X(_polyMinPointX, std::numeric_limits<double>::max());
    SET_X(_polyMaxPointX, -std::numeric_limits<double>::max());

    SET_Y(_polyMinPointY, std::numeric_limits<double>::max());
    SET_Y(_polyMaxPointY, -std::numeric_limits<double>::max());

    //getting the max and min x among vertex
    for( unsigned i = 0 ; i < readVertex.size(); ++i )
    {
        x = GET_X(readVertex[ i ]);
        y = GET_Y(readVertex[ i ]);
        
        if( x > GET_X(_polyMaxPointX) )
        {
            SET_X(_polyMaxPointX, x);
            SET_Y(_polyMaxPointX, y);
        }

        if( x < GET_X(_polyMinPointX) )
        {
            SET_X(_polyMinPointX, x);
            SET_Y(_polyMinPointX, y);
        }

        if( y > GET_Y(_polyMaxPointY) )
        {
            SET_X(_polyMaxPointY, x);
            SET_Y(_polyMaxPointY, y);
        }

        if( y < GET_Y(_polyMinPointY) )
        {
            SET_X(_polyMinPointY, x);
            SET_Y(_polyMinPointY, y);
        }
    }

}//[f]


bool getIntersectPoint(Polygon2d_t & poly, LineString2d_t & line, Point2d_t & intersectPoint, Point2d_t & contactPoint)
{
    std::vector<Point2d_t> result;

    bool valid = false;

    bool r = bg::intersection(line, poly, result);

    std::cout << "intersect points: " << std::endl;
    
    for( Point2d_t & elem : result )
    {
        std::cout << bg::dsv(elem) << ";";
    }

    std::cout << std::endl;
    
    //set the element if exist
    unsigned index = 0;

    for( auto it = result.begin(); it != result.end(); ++it )
    {
        double dist = boost::geometry::distance(*it, contactPoint);

        //skip false positive due to aproximation error
        if( dist < 0.000001 )
        {
            valid = false;
            continue;
        }
        else
        {
            SET_X(intersectPoint, GET_X(*it));
            SET_Y(intersectPoint, GET_Y(*it));
            valid = true;
            break;
        }
    }

    return valid;

}//[f]


int getPointQuadrant(Point2d_t & point)
{
    if( ( GET_X(point) > 0 ) && ( GET_Y(point) > 0 ) )
    {
        return QUAD_1;
    }
    else if( ( GET_X(point) > 0 ) && ( GET_Y(point) < 0 ) )
    {
        return QUAD_4;
    }
    else if( ( GET_X(point) < 0 ) && ( GET_Y(point) < 0 ) )
    {
        return QUAD_3;
    }
    else
    {
        return QUAD_2;
    }

    return EX_ERROR;

}//[f]


int readPolyfromFile(char * pathIn)
{
    //bring 'operator+=()' into scope
    using namespace boost::assign;

    // File pointer 
    std::fstream fin;

    // Open an existing file 
    fin.open(pathIn, std::ios::in);

    std::string line;

    std::vector<Point2d_t> vertexFromFile;

    while( std::getline(fin, line) )
    {
        std::istringstream iss(line);

        double vertX, vertY;

        if( iss >> vertX >> vertY )
        {
            vertexFromFile += Point2d_t(vertX, vertY);
        }
        else
        {
            return EX_ERROR;
        }
    }

    fin.close();

    getPolyEdgesPoints(vertexFromFile);

    getPolyExtremeVertex(vertexFromFile);

    get2PolyVertex(vertexFromFile);
  
    // Assign the points to polygon object.
    bg::assign_points(_poly, vertexFromFile);

    std::cout << "Polygon " << bg::dsv(_poly) << " has an area of " << bg::area(_poly) << std::endl;

    return EX_OK;
}//[f]


void plotContactCone()
{
    // Declare a stream and an SVG mapper
    std::ofstream svg(_imagePathNoSolution);
    bg::svg_mapper<Point2d_t> mapper(svg, 400, 400);

    mapper.add(_poly);

    mapper.add(_firstContactPoint);
    mapper.add(_secondContactPoint);
    mapper.add(_firstLineDir);
    mapper.add(_firstConeUp);
    mapper.add(_firstConeDown);


    mapper.map(_firstConeUp, "fill-opacity:0.3;fill:rgb(255,0,0);stroke:rgb(255,0,0);stroke-width:1");
    mapper.map(_firstConeDown, "fill-opacity:0.3;fill:rgb(173,255,47);stroke:rgb(173,255,47);stroke-width:1");
    mapper.map(_firstLineDir, "fill-opacity:0.3;fill:rgb(25,25,153);stroke:rgb(25,25,153);stroke-width:1");
    mapper.map(_firstContactPoint, "fill-opacity:0.5;fill:rgb(153,204,0);stroke:rgb(153,204,0);stroke-width:2", 3);
    mapper.map(_secondContactPoint, "fill-opacity:0.5;fill:rgb(153,204,0);stroke:rgb(0,0,0);stroke-width:2", 3);

    mapper.map(_poly, "fill-opacity:0.3;fill:rgb(51,51,153);stroke:rgb(51,51,153);stroke-width:2");

}//[f]



void plotContour(std::string & path)
{
    // Declare a stream and an SVG mapper
    std::ofstream svg(path);
    bg::svg_mapper<Point2d_t> mapper(svg, 400, 400);

    LineString2d_t polygonFromPoints;

    for( Point2d_t & point : _polygonPoints )
    {
        bg::append(polygonFromPoints, point);
    }
    mapper.add(polygonFromPoints);
    mapper.map(polygonFromPoints, "fill-opacity:0.3;fill:rgb(173,255,47);stroke:rgb(173,255,47);stroke-width:4");

}//[f]

void plot(bool plotVersor, bool plotContour, std::string & path)
{
    // Declare a stream and an SVG mapper
    std::ofstream svg(path);
    bg::svg_mapper<Point2d_t> mapper(svg, 400, 400);

    // Add geometries such that all these geometries fit on the map
    mapper.add(_centroid);
    mapper.add(_poly);

    mapper.add(_firstContactPoint);
    mapper.add(_secondContactPoint);

    mapper.add(_firstLineDir);
    mapper.add(_firstConeUp);
    mapper.add(_firstConeDown);    

    mapper.add(_secondLineDir);
    mapper.add(_secondConeUp);
    mapper.add(_secondConeDown);


    if( plotContour )
    {
        LineString2d_t polygonFromPoints;

        for( Point2d_t & point : _polygonPoints )
        {
            bg::append(polygonFromPoints, point);
        }
        mapper.add(polygonFromPoints);
        mapper.map(polygonFromPoints, "fill-opacity:0.3;fill:rgb(173,255,47);stroke:rgb(173,255,47);stroke-width:4");
    }

    //versor
    if( plotVersor )
    {
        LineString2d_t ver1, ver2, ver3;

        bg::append(ver1, _centroid);
        bg::append(ver1, _versDir);
        mapper.add(ver1);

        bg::append(ver2, _centroid);
        bg::append(ver2, _versFdown);
        mapper.add(ver2);

        bg::append(ver3, _centroid);
        bg::append(ver3, _versFup);
        mapper.add(ver3);

        mapper.map(ver1, "fill-opacity:0.3;fill:rgb(0,0,0);stroke:rgb(0,0,0);stroke-width:1");
        mapper.map(ver2, "fill-opacity:0.3;fill:rgb(0,0,0);stroke:rgb(0,0,0);stroke-width:1");
        mapper.map(ver3, "fill-opacity:0.3;fill:rgb(0,0,0);stroke:rgb(0,0,0);stroke-width:1");
    }



    // Draw the geometries on the SVG map, using a specific SVG style

    mapper.map(_centroid, "fill-opacity:0.5;fill:rgb(153,204,0);stroke:rgb(153,204,0);stroke-width:2", 3);
    mapper.map(_poly, "fill-opacity:0.3;fill:rgb(51,51,153);stroke:rgb(51,51,153);stroke-width:2");

    mapper.map(_firstConeUp, "fill-opacity:0.3;fill:rgb(255,0,0);stroke:rgb(255,0,0);stroke-width:1");
    mapper.map(_firstConeDown, "fill-opacity:0.3;fill:rgb(173,255,47);stroke:rgb(173,255,47);stroke-width:1");
    mapper.map(_firstLineDir, "fill-opacity:0.3;fill:rgb(25,25,153);stroke:rgb(25,25,153);stroke-width:1");

    mapper.map(_firstContactPoint, "fill-opacity:0.5;fill:rgb(153,204,0);stroke:rgb(153,204,0);stroke-width:2", 3);
    mapper.map(_secondContactPoint, "fill-opacity:0.5;fill:rgb(153,204,0);stroke:rgb(0,0,0);stroke-width:2", 3);

    mapper.map(_secondConeUp, "fill-opacity:0.3;fill:rgb(0,0,0);stroke:rgb(0,0,0);stroke-width:1");
    mapper.map(_secondConeDown, "fill-opacity:0.3;fill:rgb(0,0,0);stroke:rgb(0,0,0);stroke-width:1");
    mapper.map(_secondLineDir, "fill-opacity:0.3;fill:rgb(0,0,0);stroke:rgb(0,0,0);stroke-width:1");


    //add fake points to get margins on the plot

    Point2d_t fakeXNegative(GET_X(_polyMinPointX) - MARGIN, 0);
    Point2d_t fakeYPositive(0.0, GET_Y(_polyMaxPointY) + MARGIN);

    mapper.add(fakeXNegative);
    mapper.add(fakeYPositive);

    mapper.map(fakeXNegative, "fill-opacity:0.1;fill:rgb(0,0,0);stroke:rgb(0,0,0);stroke-width:0.5", 0.5);
    mapper.map(fakeYPositive, "fill-opacity:0.1;fill:rgb(0,0,0);stroke:rgb(0,0,0);stroke-width:0.5", 0.5);



    //rgb(173,255,47) = YELLOW
    //rgb(255,0,0)    = RED
    //rgb(0,0,0)      = BLACK
    //rgb(229,255,204)

}//[f]


double toDegrees(double angleInRadians)
{
    return static_cast<boost::units::quantity<boost::units::degree::plane_angle>>( angleInRadians * boost::units::si::radians ).value();
}//[f]


double toRadians(const double angleInDegrees)
{
    return static_cast<boost::units::quantity<boost::units::si::plane_angle>>( angleInDegrees * boost::units::degree::degrees ).value();
}//[f]

void createPath(char * folderOut)
{
    std::time_t time = std::time(0);

    std::stringstream sTime;

    sTime << time;

    std::string f(folderOut);

    _dirPath = f + "\\" + sTime.str();

    _resultPath = _dirPath + "\\" + "grasp_data_" + sTime.str() + ".txt";

    _imagePath = _dirPath + "\\" + "image_" + sTime.str() + ".svg";

    _imagePathNoSolution = _dirPath + "\\" + "no_result_" + sTime.str() + ".svg";

    boost::filesystem::path dir(_dirPath.c_str());

    if( !boost::filesystem::create_directory(dir) )
    {
        std::cout << "Error on crating directory!" << std::endl;
        return;
    }


}//[f]

double getPositiveAngle(double angle)
{
    if( angle < 0 )
    {
        return ( angle + 360 );
    }

    return angle;
}//[f]

void writeOutput()
{
    std::fstream fout;   

    // Open an existing file 
    fout.open(_resultPath, std::ios::out);

    std::stringstream stream;

    stream << (GET_X(_firstContactPoint)* MUJOCO_SCALE_FACTOR) << " " << (GET_Y(_firstContactPoint)* MUJOCO_SCALE_FACTOR) << " " << getPositiveAngle(_firstContactAngle)
        << std::endl
        << (GET_X(_secondContactPoint)* MUJOCO_SCALE_FACTOR) << " " << (GET_Y(_secondContactPoint)* MUJOCO_SCALE_FACTOR) << " " << getPositiveAngle(_secondContactAngle);

    fout << stream.str();  

    fout.close();

}//[f]