#ifndef OR_PLANNING_LOCAL_PLANNER_LINE_ITERATOR_H
#define OR_PLANNING_LOCAL_PLANNER_LINE_ITERATOR_H

#include <stdlib.h>

namespace or_local_planner {

/**
 * @brief This class do iterator operation in a grid map
 */
class FastLineIterator {
  // this method is a modified version of base_local_planner/line_iterator.h
 public:

  /**
   * @brief Class Constructor
   * @param x0 Line start x
   * @param y0 Line start y
   * @param x1 Line end x
   * @param y1 Line end y
   */
  FastLineIterator( int x0, int y0, int x1, int y1 )
      : x0_( x0 )
      , y0_( y0 )
      , x1_( x1 )
      , y1_( y1 )
      , x_( x0 )
      , y_( y0 )
      , deltax_(abs(x1 - x0))
      , deltay_(abs(y1 - y0))
      , curpixel_( 0 ) {

    xinc1_ = (x1 - x0) >0 ?1:-1;
    xinc2_ = (x1 - x0) >0 ?1:-1;

    yinc1_ = (y1 - y0) >0 ?1:-1;
    yinc2_ = (y1 - y0) >0 ?1:-1;

    if( deltax_ >= deltay_ ) {
      xinc1_ = 0;
      yinc2_ = 0;
      den_ = 2 * deltax_;
      num_ = deltax_;
      numadd_ = 2 * deltay_;
      numpixels_ = deltax_;
    } else {
      xinc2_ = 0;
      yinc1_ = 0;
      den_ = 2 * deltay_;
      num_ = deltay_;
      numadd_ = 2 * deltax_;
      numpixels_ = deltay_;
    }
  }

  ~FastLineIterator() = default;

  /**
   * @brief Line iterator's end condition
   * @return If true continue iteration, else finish line iteration
   */
  bool IsValid() const {
    return curpixel_ <= numpixels_;
  }

  /**
   * @brief Do add iteration add
   */
  void Advance() {
    num_ += numadd_;
    if( num_ >= den_ ) {
      num_ -= den_;
      x_ += xinc1_;
      y_ += yinc1_;
    }
    x_ += xinc2_;
    y_ += yinc2_;

    curpixel_++;
  }

  /**
   * @brief Get current x
   * @return Current x
   */
  int GetX() const { return x_; }
  /**
   * @brief Get current y
   * @return Current y
   */
  int GetY() const { return y_; }

  /**
   * @brief Get start x
   * @return Start x
   */
  int GetX0() const { return x0_; }
  /**
   * @brief Get start y
   * @return Start y
   */
  int GetY0() const { return y0_; }

  /**
   * @brief Get end x
   * @return End x
   */
  int GetX1() const { return x1_; }
  /**
   * @brief Get end y
   * @return End y
   */
  int GetY1() const { return y1_; }

 private:
  //! x0_ Line start x
  int x0_;
  //! y0_ Line start y
  int y0_;
  //! x1_ Line end x
  int x1_;
  //! y1_ Line end y
  int y1_;

  //! Iterator value
  int x_;
  int y_;

  //! total change
  int deltax_;
  int deltay_;

  //! current iterator steep
  int curpixel_;

  //! xinc1_, yinc1_ x_ and y_ increase value if num_ > den_
  //! xinc2_, yinc2_ x_ and y_ increase value if num_ < den_
  int xinc1_, xinc2_, yinc1_, yinc2_;

  //! numpixels_ total iterator steep
  //! num_ current error
  //! numadd_ error should add after a iterator
  //! den_ max error
  int den_, num_, numadd_, numpixels_;
};


} // namespace or_local_planner

#endif //or_PLANNING_LOCAL_PLANNER_LINE_ITERATOR_H

