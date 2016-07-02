/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright (C) 2013 CERN
 * @author Tomasz Wlostowski <tomasz.wlostowski@cern.ch>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you may find one here:
 * http://www.gnu.org/licenses/old-licenses/gpl-2.0.html
 * or you may search the http://www.gnu.org website for the version 2 license,
 * or you may write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
 */

/**
 * @file profile.h:
 * @brief Simple profiling functions for measuring code execution time.
 */

#ifndef __TPROFILE_H
#define __TPROFILE_H

#include <chrono>

using profile_clock = std::chrono::high_resolution_clock;

/**
 * Function get_tics
 * Returns the number of microseconds that have elapsed since the system was started.
 * @return uint64_t Number of microseconds.
 */
static inline profile_clock::time_point get_tics()
{
    return profile_clock::now();
}

/**
 * Structure for storing data related to profiling counters.
 */
struct prof_counter
{
    profile_clock::time_point start, end;         // Stored timer value

    uint64_t usecs() const
    {
        return std::chrono::duration_cast<std::chrono::microseconds>( end - start ).count();
    }

    float msecs() const
    {
        return std::chrono::duration_cast<std::chrono::duration<float, std::milli>>( end - start ).count();
    }
};

/**
 * Function prof_start
 * Begins code execution time counting for a given profiling counter.
 * @param aCnt is the counter which should be started.
 * use_rdtsc tells if processor's time-stamp counter should be used for time counting.
 *      Otherwise is system tics method will be used. IMPORTANT: time-stamp counter should not
 *      be used on multicore machines executing threaded code.
 */
static inline void prof_start( prof_counter* aCnt )
{
    aCnt->start = get_tics();
}

/**
 * Function prof_stop
 * Ends code execution time counting for a given profiling counter.
 * @param aCnt is the counter which should be stopped.
 */
static inline void prof_end( prof_counter* aCnt )
{
    aCnt->end = get_tics();
}

#endif
