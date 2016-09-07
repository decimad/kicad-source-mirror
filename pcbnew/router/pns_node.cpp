/*
 * KiRouter - a push-and-(sometimes-)shove PCB router
 *
 * Copyright (C) 2013-2014 CERN
 * Copyright (C) 2016 KiCad Developers, see AUTHORS.txt for contributors.
 * Author: Tomasz Wlostowski <tomasz.wlostowski@cern.ch>
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <vector>
#include <cassert>

#include <math/vector2d.h>

#include <geometry/seg.h>
#include <geometry/shape.h>
#include <geometry/shape_line_chain.h>
#include <geometry/shape_index.h>

#include "pns_item.h"
#include "pns_line.h"
#include "pns_node.h"
#include "pns_via.h"
#include "pns_solid.h"
#include "pns_joint.h"
#include "pns_index.h"
#include "pns_router.h"

using boost::unordered_set;
using boost::unordered_map;

namespace PNS {

NODE::NODE( REVISION* aRevision )
    : m_revision( aRevision )
{
    wxLogTrace( "PNS", "NODE::create %p", this );
    m_maxClearance = 800000;    // fixme: depends on how thick traces are.
    m_ruleResolver = NULL;
}


NODE::~NODE()
{
    wxLogTrace( "PNS", "NODE::delete %p", this );
}

int NODE::GetClearance( const ITEM* aA, const ITEM* aB ) const
{
   if( !m_ruleResolver )
        return 100000;

   return m_ruleResolver->Clearance( aA, aB );
}

// ================
// Revision Methods
// ================

REVISION* NODE::GetRevision()
{
    return m_revision;
}

REVISION_PATH NODE::Path( REVISION* aAncestor ) const
{
    return m_revision->Path( aAncestor );
}

CHANGE_SET NODE::GetRevisionChanges() const
{
    return m_revision->GetRevisionChanges();
}

REVISION* NODE::BranchMove()
{
    REVISION* result = m_revision;
    m_revision = m_revision->Branch();
    return result;
}

void NODE::Squash()
{
    m_revision = m_revision->Squash();
}

void NODE::SquashToRevision( REVISION* aAncestor )
{
    while( m_revision != aAncestor )
    {
        Squash();
    }
}

void NODE::SquashToParentRevision( REVISION* aAncestor )
{
    while( m_revision->Parent() != aAncestor )
    {
        Squash();
    }
}

void NODE::Revert()
{
    revertRevision( m_revision );
    m_revision = m_revision->Revert();
}

void NODE::RevertToRevision( REVISION* aAncestor )
{
    while( m_revision != aAncestor )
    {
        Revert();
    }
}

void NODE::RevertToParentRevision( REVISION* aAncestor )
{
    while( m_revision->Parent() != aAncestor )
    {
        Revert();
    }
}

void NODE::CheckoutRevision( const REVISION* aRevision )
{
    WalkPath( PNS::Path( m_revision, aRevision ) );
}

void NODE::WalkPath( const REVISION_PATH& aPath )
{
    for( auto revision : aPath.RevertSequence() )
    {
        assert(revision == m_revision);
        revertRevision( revision );
        m_revision = m_revision->Parent();
    }

    for( auto revision : aPath.ApplySequence() )
    {
        assert( revision->Parent() == m_revision );
        applyRevision( revision );
        // The paths are intended to hold const, we could translate the
        // the pointer to non-const by searching the revision's branches,
        // but given the calling code is wellformed (the above assertion
        // holds), it would result in the same result as a const_cast.
        m_revision = const_cast<REVISION*>(revision);
    }
}

void NODE::ClearBranches()
{
    m_revision->ClearBranches();
}

void NODE::applyRevision( const REVISION* aRevision )
{
    for( auto item_ptr : aRevision->RemovedItems() )
    {
        removeItemIndex( item_ptr );
    }

    for( auto& item_ptr : aRevision->AddedItems() )
    {
        addItemIndex( item_ptr.get() );
    }
}

void NODE::revertRevision( const REVISION* aRevision )
{
    for( auto& item_ptr : aRevision->AddedItems() )
    {
        removeItemIndex( item_ptr.get() );
    }

    for( auto item_ptr : aRevision->RemovedItems() )
    {
        addItemIndex( item_ptr );
    }
}

void NODE::Clear()
{
    m_index.Clear();
    m_joints.clear();
    m_revision->Clear();
}

// =============
// Index Methods
// =============

OBSTACLE_VISITOR::OBSTACLE_VISITOR( const ITEM* aItem ) :
    m_item( aItem ),
    m_node( NULL ),
    m_extraClearance( 0 )
{

}


void OBSTACLE_VISITOR::SetWorld( const NODE* aNode )
{
    m_node = aNode;
}


bool OBSTACLE_VISITOR::visit( ITEM* aCandidate )
{
/*
    // check if there is a more recent branch with a newer
    // (possibily modified) version of this item.
    if( m_override && m_override->Overrides( aCandidate ) )
        return true;
*/
    return false;
}


// function object that visits potential obstacles and performs
// the actual collision refining
struct NODE::DEFAULT_OBSTACLE_VISITOR : public OBSTACLE_VISITOR
{
    ///> list of encountered obstacles
    OBSTACLES& m_tab;

    ///> acccepted kinds of colliding items (solids, vias, segments, etc...)
    int m_kindMask;

    ///> max number of hits
    int m_limitCount;

    ///> number of items found so far
    int m_matchCount;

    ///> additional clearance
    int m_extraClearance;

    bool m_differentNetsOnly;

    int m_forceClearance;

    DEFAULT_OBSTACLE_VISITOR( NODE::OBSTACLES& aTab, const ITEM* aItem, int aKindMask, bool aDifferentNetsOnly ) :
        OBSTACLE_VISITOR( aItem ),
        m_tab( aTab ),
        m_kindMask( aKindMask ),
        m_limitCount( -1 ),
        m_matchCount( 0 ),
        m_extraClearance( 0 ),
        m_differentNetsOnly( aDifferentNetsOnly ),
        m_forceClearance( -1 )
    {
        if( aItem && aItem->Kind() == ITEM::LINE_T )
        {
             m_extraClearance += static_cast<const LINE*>( aItem )->Width() / 2;
        }
    }

    void SetCountLimit( int aLimit )
    {
        m_limitCount = aLimit;
    }

    bool operator()( ITEM* aCandidate )
    {
        if( !aCandidate->OfKind( m_kindMask ) )
            return true;

        if( visit( aCandidate ) )
            return true;

        int clearance = m_extraClearance + m_node->GetClearance( aCandidate, m_item );

        if( aCandidate->Kind() == ITEM::LINE_T ) // this should never happen.
        {
            assert( false );
            clearance += static_cast<LINE*>( aCandidate )->Width() / 2;
        }

        if( m_forceClearance >= 0 )
            clearance = m_forceClearance;

        if( !aCandidate->Collide( m_item, clearance, m_differentNetsOnly ) )
            return true;

        OBSTACLE obs;

        obs.m_item = aCandidate;
        obs.m_head = m_item;
        m_tab.push_back( obs );

        m_matchCount++;

        if( m_limitCount > 0 && m_matchCount >= m_limitCount )
            return false;

        return true;
    };
};


int NODE::QueryColliding( const ITEM* aItem, OBSTACLE_VISITOR& aVisitor )
{
    aVisitor.SetWorld( this );
    m_index.Query( aItem, m_maxClearance, aVisitor );
    return 0;
}


int NODE::QueryColliding( const ITEM* aItem,
        NODE::OBSTACLES& aObstacles, int aKindMask, int aLimitCount, bool aDifferentNetsOnly, int aForceClearance )
{
    DEFAULT_OBSTACLE_VISITOR visitor( aObstacles, aItem, aKindMask, aDifferentNetsOnly );

#ifdef DEBUG
    assert( allocNodes.find( this ) != allocNodes.end() );
#endif

    visitor.SetCountLimit( aLimitCount );
    visitor.SetWorld( this );
    visitor.m_forceClearance = aForceClearance;
    m_index.Query( aItem, m_maxClearance, visitor );

    return aObstacles.size();
}


NODE::OPT_OBSTACLE NODE::NearestObstacle( const LINE* aItem, int aKindMask,
                                                  const std::set<ITEM*>* aRestrictedSet )
{
    OBSTACLES obs_list;
    bool found_isects = false;

    const SHAPE_LINE_CHAIN& line = aItem->CLine();

    obs_list.reserve( 100 );

    int n = 0;

    for( int i = 0; i < line.SegmentCount(); i++ )
    {
        const SEGMENT s( *aItem, line.CSegment( i ) );
        n += QueryColliding( &s, obs_list, aKindMask );
    }

    if( aItem->EndsWithVia() )
        n += QueryColliding( &aItem->Via(), obs_list, aKindMask );

    if( !n )
        return OPT_OBSTACLE();

    LINE& aLine = (LINE&) *aItem;

    OBSTACLE nearest;
    nearest.m_item = NULL;
    nearest.m_distFirst = INT_MAX;

    for( OBSTACLE obs : obs_list )
    {
        VECTOR2I ip_first, ip_last;
        int dist_max = INT_MIN;

        if( aRestrictedSet && aRestrictedSet->find( obs.m_item ) == aRestrictedSet->end() )
            continue;

        std::vector<SHAPE_LINE_CHAIN::INTERSECTION> isect_list;

        int clearance = GetClearance( obs.m_item, &aLine );

        SHAPE_LINE_CHAIN hull = obs.m_item->Hull( clearance, aItem->Width() );

        if( aLine.EndsWithVia() )
        {
            int clearance = GetClearance( obs.m_item, &aLine.Via() );

            SHAPE_LINE_CHAIN viaHull = aLine.Via().Hull( clearance, aItem->Width() );

            viaHull.Intersect( hull, isect_list );

            for( SHAPE_LINE_CHAIN::INTERSECTION isect : isect_list )
            {
                int dist = aLine.CLine().Length() +
                           ( isect.p - aLine.Via().Pos() ).EuclideanNorm();

                if( dist < nearest.m_distFirst )
                {
                    found_isects = true;
                    nearest.m_distFirst = dist;
                    nearest.m_ipFirst = isect.p;
                    nearest.m_item = obs.m_item;
                    nearest.m_hull = hull;
                }

                if( dist > dist_max )
                {
                    dist_max = dist;
                    ip_last = isect.p;
                }
            }
        }

        isect_list.clear();

        hull.Intersect( aLine.CLine(), isect_list );

        for( SHAPE_LINE_CHAIN::INTERSECTION isect : isect_list )
        {
            int dist = aLine.CLine().PathLength( isect.p );

            if( dist < nearest.m_distFirst )
            {
                found_isects = true;
                nearest.m_distFirst = dist;
                nearest.m_ipFirst = isect.p;
                nearest.m_item = obs.m_item;
                nearest.m_hull = hull;
            }

            if( dist > dist_max )
            {
                dist_max = dist;
                ip_last = isect.p;
            }
        }

        nearest.m_ipLast = ip_last;
        nearest.m_distLast = dist_max;
    }

    if( !found_isects )
        nearest.m_item = obs_list[0].m_item;

    return nearest;
}


NODE::OPT_OBSTACLE NODE::CheckColliding( const ITEM_SET& aSet, int aKindMask )
{
    for( const ITEM* item : aSet.CItems() )
    {
        OPT_OBSTACLE obs = CheckColliding( item, aKindMask );

        if( obs )
            return  obs;
    }

    return OPT_OBSTACLE();
}


NODE::OPT_OBSTACLE NODE::CheckColliding( const ITEM* aItemA, int aKindMask )
{
    OBSTACLES obs;

    obs.reserve( 100 );

    if( aItemA->Kind() == ITEM::LINE_T )
    {
        int n = 0;
        const LINE* line = static_cast<const LINE*>( aItemA );
        const SHAPE_LINE_CHAIN& l = line->CLine();

        for( int i = 0; i < l.SegmentCount(); i++ )
        {
            const SEGMENT s( *line, l.CSegment( i ) );
            n += QueryColliding( &s, obs, aKindMask, 1 );

            if( n )
                return OPT_OBSTACLE( obs[0] );
        }

        if( line->EndsWithVia() )
        {
            n += QueryColliding( &line->Via(), obs, aKindMask, 1 );

            if( n )
                return OPT_OBSTACLE( obs[0] );
        }
    }
    else if( QueryColliding( aItemA, obs, aKindMask, 1 ) > 0 )
        return OPT_OBSTACLE( obs[0] );

    return OPT_OBSTACLE();
}


bool NODE::CheckColliding( const ITEM* aItemA, const ITEM* aItemB, int aKindMask, int aForceClearance )
{
    assert( aItemB );
    int clearance;
    if( aForceClearance >= 0 )
        clearance = aForceClearance;
    else
        clearance = GetClearance( aItemA, aItemB );

    // fixme: refactor
    if( aItemA->Kind() == ITEM::LINE_T )
        clearance += static_cast<const LINE*>( aItemA )->Width() / 2;
    if( aItemB->Kind() == ITEM::LINE_T )
        clearance += static_cast<const LINE*>( aItemB )->Width() / 2;

    return aItemA->Collide( aItemB, clearance );
}


struct HIT_VISITOR : public OBSTACLE_VISITOR
{
    ITEM_SET& m_items;
    const VECTOR2I& m_point;

    HIT_VISITOR( ITEM_SET& aTab, const VECTOR2I& aPoint ) :
        OBSTACLE_VISITOR( NULL ),
        m_items( aTab ), m_point( aPoint )
    {}

    bool operator()( ITEM* aItem )
    {
        SHAPE_CIRCLE cp( m_point, 0 );

        int cl = 0;

        if( aItem->Shape()->Collide( &cp, cl ) )
            m_items.Add( aItem );

        return true;
    }
};


const ITEM_SET NODE::HitTest( const VECTOR2I& aPoint ) const
{
    ITEM_SET items;

    // fixme: we treat a point as an infinitely small circle - this is inefficient.
    SHAPE_CIRCLE s( aPoint, 0 );
    HIT_VISITOR visitor( items, aPoint );
    visitor.SetWorld( this );

    m_index.Query( &s, m_maxClearance, visitor );

    return items;
}


void NODE::addSolidIndex( SOLID* aSolid )
{
    linkJoint( aSolid->Pos(), aSolid->Layers(), aSolid->Net(), aSolid );
    m_index.Add( aSolid );
}

void NODE::Add( std::unique_ptr< SOLID > aSolid )
{
    addSolidIndex( aSolid.get() );
    m_revision->AddItem( std::move( aSolid ) );
}

void NODE::addViaIndex( VIA* aVia )
{
    linkJoint( aVia->Pos(), aVia->Layers(), aVia->Net(), aVia );
    m_index.Add( aVia );
}

void NODE::Add( std::unique_ptr< VIA > aVia )
{
    addViaIndex( aVia.get() );
    m_revision->AddItem( std::move( aVia ) );
}

void NODE::Add( LINE& aLine, bool aAllowRedundant )
{
    assert( !aLine.IsLinked() );

    SHAPE_LINE_CHAIN& l = aLine.Line();

    for( int i = 0; i < l.SegmentCount(); i++ )
    {
        SEG s = l.CSegment( i );

        if( s.A != s.B )
        {
            SEGMENT* rseg;
            if( !aAllowRedundant &&
                (rseg = findRedundantSegment( s.A, s.B, aLine.Layers(), aLine.Net() )) )
            {
                // another line could be referencing this segment too :(
                aLine.LinkSegment( rseg );
            }
            else
            {
                std::unique_ptr< SEGMENT > newseg( new SEGMENT( aLine, s ) );
                aLine.LinkSegment( newseg.get() );
                Add( std::move( newseg ), true );
            }
        }
    }
    aLine.SetOwner( GetRevision() );
}

void NODE::addSegmentIndex( SEGMENT* aSeg )
{
    linkJoint( aSeg->Seg().A, aSeg->Layers(), aSeg->Net(), aSeg );
    linkJoint( aSeg->Seg().B, aSeg->Layers(), aSeg->Net(), aSeg );
    m_index.Add( aSeg );
}

void NODE::Add( std::unique_ptr< SEGMENT > aSegment, bool aAllowRedundant )
{
    if( aSegment->Seg().A == aSegment->Seg().B )
    {
        wxLogTrace( "PNS", "attempting to add a segment with same end coordinates, ignoring." );
        return;
    }

    if( !aAllowRedundant && findRedundantSegment( aSegment.get() ) )
        return;

    addSegmentIndex( aSegment.get() );
    m_revision->AddItem( std::move( aSegment ) );
}

void NODE::Add( std::unique_ptr< ITEM > aItem, bool aAllowRedundant )
{
    switch( aItem->Kind() )
    {
    case ITEM::SOLID_T:
        Add( ItemCast<SOLID>( std::move( aItem ) ) );
        break;

    case ITEM::SEGMENT_T:
        Add( ItemCast<SEGMENT>( std::move( aItem ) ), aAllowRedundant );
        break;

    case ITEM::LINE_T:
        assert( false );
        break;

    case ITEM::VIA_T:
        Add( ItemCast<VIA>( std::move( aItem ) ) );
        break;

    default:
        assert( false );
    }
}

void NODE::addItemIndex( ITEM* aItem )
{
    switch( aItem->Kind() )
    {
    case ITEM::SOLID_T:
        addSolidIndex( static_cast<SOLID*>( aItem ) );
        break;

    case ITEM::SEGMENT_T:
        addSegmentIndex( static_cast<SEGMENT*>( aItem ) );
        break;

    case ITEM::LINE_T:
        assert( false );
        break;

    case ITEM::VIA_T:
        addViaIndex( static_cast<VIA*>( aItem ) );
        break;

    default:
        assert( false );
    }
}

void NODE::removeSegmentIndex( SEGMENT* aSeg )
{
    unlinkJoint( aSeg->Seg().A, aSeg->Layers(), aSeg->Net(), aSeg );
    unlinkJoint( aSeg->Seg().B, aSeg->Layers(), aSeg->Net(), aSeg );
    m_index.Remove( aSeg );
}

void NODE::removeViaIndex( VIA* aVia )
{
    // We have to split a single joint (associated with a via, binding together multiple layers)
    // into multiple independent joints. As I'm a lazy bastard, I simply delete the via and all its links and re-insert them.

    JOINT::HASH_TAG tag;

    VECTOR2I p( aVia->Pos() );
    LAYER_RANGE vLayers( aVia->Layers() );
    int net = aVia->Net();

    JOINT* jt = FindJoint( p, vLayers.Start(), net );
    JOINT::LINKED_ITEMS links( jt->LinkList() );

    tag.net = net;
    tag.pos = p;

    bool split;
    do
    {
        split = false;
        std::pair<JOINT_MAP::iterator, JOINT_MAP::iterator> range = m_joints.equal_range( tag );

        if( range.first == m_joints.end() )
            break;

        // find and remove all joints containing the via to be removed

        for( JOINT_MAP::iterator f = range.first; f != range.second; ++f )
        {
            if( aVia->LayersOverlap( &f->second ) )
            {
                m_joints.erase( f );
                split = true;
                break;
            }
        }
    } while( split );

    // and re-link them, using the former via's link list
    for(ITEM* item : links)
    {
        if( item != aVia )
            linkJoint( p, item->Layers(), net, item );
    }

    m_index.Remove( aVia );
}

void NODE::removeSolidIndex( SOLID* aSolid )
{
    // fixme: this fucks up the joints, but it's only used for marking colliding obstacles for the moment, so we don't care.
    unlinkJoint( aSolid->Pos(), aSolid->Layers(), aSolid->Net(), aSolid );
    m_index.Remove( aSolid );
}


void NODE::Replace( ITEM* aOldItem, std::unique_ptr< ITEM > aNewItem )
{
    Remove( aOldItem );
    Add( std::move( aNewItem ) );
}

void NODE::Replace( LINE& aOldLine, LINE& aNewLine )
{
    Remove( aOldLine );
    Add( aNewLine );
}

void NODE::Remove( SOLID* aSolid )
{
    removeSolidIndex( aSolid );
    m_revision->RemoveItem( aSolid );
}

void NODE::Remove( VIA* aVia )
{
    removeViaIndex( aVia );
    m_revision->RemoveItem( aVia );
}

void NODE::Remove( SEGMENT* aSegment )
{
    removeSegmentIndex( aSegment );
    m_revision->RemoveItem( aSegment );
}

void NODE::removeItemIndex( ITEM* aItem )
{
    switch( aItem->Kind() )
    {
    case ITEM::SOLID_T:
        removeSolidIndex( static_cast<SOLID*>(aItem) );
        break;

    case ITEM::SEGMENT_T:
        removeSegmentIndex( static_cast<SEGMENT*>(aItem) );
        break;

    case ITEM::LINE_T:
        assert( false );
        break;

    case ITEM::VIA_T:
        removeViaIndex( static_cast<VIA*>(aItem) );
        break;

    default:
        break;
    }
}

void NODE::Remove( ITEM* aItem )
{
    removeItemIndex( aItem );
    m_revision->RemoveItem( aItem );
}


void NODE::Remove( LINE& aLine )
{
    // LINE does not have a seperate remover, as LINEs are never truly a member of the tree
    std::vector<SEGMENT*>& segRefs = aLine.LinkedSegments();

    for( SEGMENT* seg : segRefs )
    {
        Remove( seg );
    }

    aLine.SetOwner( nullptr );
    aLine.ClearSegmentLinks();
}


void NODE::followLine( SEGMENT* aCurrent, bool aScanDirection, int& aPos,
        int aLimit, VECTOR2I* aCorners, SEGMENT** aSegments, bool& aGuardHit,
        bool aStopAtLockedJoints )
{
    bool prevReversed = false;

    const VECTOR2I guard = aScanDirection ? aCurrent->Seg().B : aCurrent->Seg().A;

    for( int count = 0 ; ; ++count )
    {
        const VECTOR2I p =
            ( aScanDirection ^ prevReversed ) ? aCurrent->Seg().B : aCurrent->Seg().A;
        const JOINT* jt = FindJoint( p, aCurrent );

        assert( jt );

        aCorners[aPos] = jt->Pos();
        aSegments[aPos] = aCurrent;
        aPos += ( aScanDirection ? 1 : -1 );

        if( count && guard == p)
        {
            aSegments[aPos] = NULL;
            aGuardHit = true;
            break;
        }

        bool locked = aStopAtLockedJoints ? jt->IsLocked() : false;

        if( locked || !jt->IsLineCorner() || aPos < 0 || aPos == aLimit )
            break;

        aCurrent = jt->NextSegment( aCurrent );

        prevReversed =
            ( jt->Pos() == ( aScanDirection ? aCurrent->Seg().B : aCurrent->Seg().A ) );
    }
}


const LINE NODE::AssembleLine( SEGMENT* aSeg, int* aOriginSegmentIndex, bool aStopAtLockedJoints )
{
    const int MaxVerts = 1024 * 16;

    VECTOR2I corners[MaxVerts + 1];
    SEGMENT* segs[MaxVerts + 1];

    LINE pl;
    bool guardHit = false;

    int i_start = MaxVerts / 2, i_end = i_start + 1;

    pl.SetWidth( aSeg->Width() );
    pl.SetLayers( aSeg->Layers() );
    pl.SetNet( aSeg->Net() );
    pl.SetOwner( GetRevision() );

    followLine( aSeg, false, i_start, MaxVerts, corners, segs, guardHit, aStopAtLockedJoints );

    if( !guardHit )
        followLine( aSeg, true, i_end, MaxVerts, corners, segs, guardHit, aStopAtLockedJoints );

    int n = 0;

    SEGMENT* prev_seg = NULL;
    bool originSet = false;

    for( int i = i_start + 1; i < i_end; i++ )
    {
        const VECTOR2I& p = corners[i];

        pl.Line().Append( p );

        if( segs[i] && prev_seg != segs[i] )
        {
            pl.LinkSegment( segs[i] );

            // latter condition to avoid loops
            if( segs[i] == aSeg && aOriginSegmentIndex && !originSet )
            {
                *aOriginSegmentIndex = n;
                originSet = true;
            }
            n++;
        }

        prev_seg = segs[i];
    }

    assert( pl.SegmentCount() != 0 );

    return pl;
}


void NODE::FindLineEnds( const LINE& aLine, JOINT& aA, JOINT& aB )
{
    aA = *FindJoint( aLine.CPoint( 0 ), &aLine );
    aB = *FindJoint( aLine.CPoint( -1 ), &aLine );
}


#if 0
void NODE::MapConnectivity( JOINT* aStart, std::vector<JOINT*>& aFoundJoints )
{
    std::deque<JOINT*> searchQueue;
    std::set<JOINT*> processed;

    searchQueue.push_back( aStart );
    processed.insert( aStart );

    while( !searchQueue.empty() )
    {
        JOINT* current = searchQueue.front();
        searchQueue.pop_front();

        for( ITEM* item : current->LinkList() )
        {
            if( item->OfKind( ITEM::SEGMENT_T ) )
            {
                SEGMENT* seg = static_cast<SEGMENT *>( item );
                JOINT* a = FindJoint( seg->Seg().A, seg );
                JOINT* b = FindJoint( seg->Seg().B, seg );
                JOINT* next = ( *a == *current ) ? b : a;

                if( processed.find( next ) == processed.end() )
                {
                    processed.insert( next );
                    searchQueue.push_back( next );
                }
            }
        }
    }

    for(JOINT* jt : processed)
        aFoundJoints.push_back( jt );
}
#endif


int NODE::FindLinesBetweenJoints( JOINT& aA, JOINT& aB, std::vector<LINE>& aLines )
{
    for( ITEM* item : aA.LinkList() )
    {
        if( item->Kind() == ITEM::SEGMENT_T )
        {
            SEGMENT* seg = static_cast<SEGMENT*>( item );
            LINE line = AssembleLine( seg );

            if( !line.Layers().Overlaps( aB.Layers() ) )
                continue;

            JOINT j_start, j_end;

            FindLineEnds( line, j_start, j_end );

            int id_start = line.CLine().Find( aA.Pos() );
            int id_end   = line.CLine().Find( aB.Pos() );

            if( id_end < id_start )
                std::swap( id_end, id_start );

            if( id_start >= 0 && id_end >= 0 )
            {
                line.ClipVertexRange( id_start, id_end );
                aLines.push_back( line );
            }
        }
    }

    return 0;
}


JOINT* NODE::FindJoint( const VECTOR2I& aPos, int aLayer, int aNet )
{
    JOINT::HASH_TAG tag;

    tag.net = aNet;
    tag.pos = aPos;

    JOINT_MAP::iterator f = m_joints.find( tag ), end = m_joints.end();

    if( f == end )
        return NULL;

    while( f != end )
    {
        if( f->second.Layers().Overlaps( aLayer ) )
            return &f->second;

        ++f;
    }

    return NULL;
}


void NODE::LockJoint( const VECTOR2I& aPos, const ITEM* aItem, bool aLock )
{
    JOINT& jt = touchJoint( aPos, aItem->Layers(), aItem->Net() );
    jt.Lock( aLock );
}


JOINT& NODE::touchJoint( const VECTOR2I& aPos, const LAYER_RANGE& aLayers, int aNet )
{
    JOINT::HASH_TAG tag{ aPos, aNet };
    using JOINT_ITERATOR = JOINT_MAP::iterator;

    // now insert and combine overlapping joints
    JOINT jt( aPos, aLayers, aNet );

    bool merged;

    do
    {
        std::pair<JOINT_ITERATOR, JOINT_ITERATOR> range;
        merged  = false;
        range   = m_joints.equal_range( tag );

        if( range.first == m_joints.end() )
            break;

        for( JOINT_ITERATOR f = range.first; f != range.second; ++f )
        {
            if( aLayers.Overlaps( f->second.Layers() ) )
            {
                jt.Merge( f->second );
                m_joints.erase( f );
                merged = true;
                break;
            }
        }
    }
    while( merged );

    return m_joints.insert( TagJointPair( tag, jt ) )->second;
}


void JOINT::Dump() const
{
    wxLogTrace( "PNS", "joint layers %d-%d, net %d, pos %s, links: %d", m_layers.Start(),
            m_layers.End(), m_tag.net, m_tag.pos.Format().c_str(), LinkCount() );
}


void NODE::linkJoint( const VECTOR2I& aPos, const LAYER_RANGE& aLayers,
                          int aNet, ITEM* aWhere )
{
    JOINT& jt = touchJoint( aPos, aLayers, aNet );

    jt.Link( aWhere );
}


void NODE::unlinkJoint( const VECTOR2I& aPos, const LAYER_RANGE& aLayers,
                            int aNet, ITEM* aWhere )
{
    // fixme: remove dangling joints
    JOINT& jt = touchJoint( aPos, aLayers, aNet );

    jt.Unlink( aWhere );
}


void NODE::Dump( bool aLong )
{
#if 0
    boost::unordered_set<SEGMENT*> all_segs;
    SHAPE_INDEX_LIST<ITEM*>::iterator i;

    for( i = m_items.begin(); i != m_items.end(); i++ )
    {
        if( (*i)->GetKind() == ITEM::SEGMENT_T )
            all_segs.insert( static_cast<SEGMENT*>( *i ) );
    }

    if( !isRoot() )
    {
        for( i = m_root->m_items.begin(); i != m_root->m_items.end(); i++ )
        {
            if( (*i)->GetKind() == ITEM::SEGMENT_T && !overrides( *i ) )
                all_segs.insert( static_cast<SEGMENT*>(*i) );
        }
    }

    JOINT_MAP::iterator j;

    if( aLong )
        for( j = m_joints.begin(); j != m_joints.end(); ++j )
        {
            wxLogTrace( "PNS", "joint : %s, links : %d\n",
                    j->second.GetPos().Format().c_str(), j->second.LinkCount() );
            JOINT::LINKED_ITEMS::const_iterator k;

            for( k = j->second.GetLinkList().begin(); k != j->second.GetLinkList().end(); ++k )
            {
                const ITEM* m_item = *k;

                switch( m_item->GetKind() )
                {
                case ITEM::SEGMENT_T:
                    {
                        const SEGMENT* seg = static_cast<const SEGMENT*>( m_item );
                        wxLogTrace( "PNS", " -> seg %s %s\n", seg->GetSeg().A.Format().c_str(),
                                seg->GetSeg().B.Format().c_str() );
                        break;
                    }

                default:
                    break;
                }
            }
        }


    int lines_count = 0;

    while( !all_segs.empty() )
    {
        SEGMENT* s = *all_segs.begin();
        LINE* l = AssembleLine( s );

        LINE::LinkedSegments* seg_refs = l->GetLinkedSegments();

        if( aLong )
            wxLogTrace( "PNS", "Line: %s, net %d ", l->GetLine().Format().c_str(), l->GetNet() );

        for( std::vector<SEGMENT*>::iterator j = seg_refs->begin(); j != seg_refs->end(); ++j )
        {
            wxLogTrace( "PNS", "%s ", (*j)->GetSeg().A.Format().c_str() );

            if( j + 1 == seg_refs->end() )
                wxLogTrace( "PNS", "%s\n", (*j)->GetSeg().B.Format().c_str() );

            all_segs.erase( *j );
        }

        lines_count++;
    }

    wxLogTrace( "PNS", "Local joints: %d, lines : %d \n", m_joints.size(), lines_count );
#endif
}

void NODE::AllItemsInNet( int aNet, std::set<ITEM*>& aItems )
{
    INDEX::NET_ITEMS_LIST* l_cur = m_index.GetItemsForNet( aNet );

    if( l_cur )
    {
        for( ITEM*item : *l_cur )
            aItems.insert( item );
    }
}


void NODE::ClearRanks( int aMarkerMask )
{
    for( auto& itemptr : m_index )
    {
        itemptr->SetRank( -1 );
        itemptr->Mark( itemptr->Marker() & (~aMarkerMask) );
    }
}


int NODE::FindByMarker( int aMarker, ITEM_SET& aItems )
{
    for( auto& itemptr : m_index )
    {
        if( itemptr->Marker() & aMarker )
            aItems.Add( itemptr );
    }

    return 0;
}


int NODE::RemoveByMarker( int aMarker )
{
    std::vector<ITEM*> garbage;

    for( auto& itemptr : m_index )
    {
        if( itemptr->Marker() & aMarker )
        {
            garbage.push_back( itemptr );
        }
    }

    for( auto garbage_item : garbage )
    {
        Remove( garbage_item );
    }

    return 0;
}

SEGMENT* NODE::findRedundantSegment( const VECTOR2I& A, const VECTOR2I& B, const LAYER_RANGE& lr,
                                     int aNet )
{
    JOINT* jtStart = FindJoint( A, lr.Start(), aNet );

    if( !jtStart )
        return nullptr;

    for( ITEM* item : jtStart->LinkList() )
    {
        if( item->OfKind( ITEM::SEGMENT_T ) )
        {
            SEGMENT* seg2 = (SEGMENT*)item;

            const VECTOR2I a2( seg2->Seg().A );
            const VECTOR2I b2( seg2->Seg().B );

            if( seg2->Layers().Start() == lr.Start() &&
                ((A == a2 && B == b2) || (A == b2 && B == a2)) )
                return seg2;
        }
    }

    return nullptr;
}

SEGMENT* NODE::findRedundantSegment( SEGMENT* aSeg )
{
    return findRedundantSegment( aSeg->Seg().A, aSeg->Seg().B, aSeg->Layers(), aSeg->Net() );
}


ITEM *NODE::FindItemByParent( const BOARD_CONNECTED_ITEM* aParent )
{
    INDEX::NET_ITEMS_LIST* l_cur = m_index.GetItemsForNet( aParent->GetNetCode() );

    for( ITEM* item : *l_cur )
        if( item->Parent() == aParent )
            return item;

    return NULL;
}

}
