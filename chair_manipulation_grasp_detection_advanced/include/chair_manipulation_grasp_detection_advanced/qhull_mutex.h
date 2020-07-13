#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_QHULL_MUTEX_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_QHULL_MUTEX_H

#include <qmutex.h>

namespace chair_manipulation
{
/**
 * The global mutex used to synchronize access to QHull
 */
extern QMutex qhull_mutex;

class QhullLockGuard
{
public:
  QhullLockGuard()
  {
    qhull_mutex.lock();
  }

  ~QhullLockGuard()
  {
    qhull_mutex.unlock();
  }
};

}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_QHULL_MUTEX_H
