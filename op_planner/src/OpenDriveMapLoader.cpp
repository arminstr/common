/*
 * OpenDriveMapLoader.cpp
 *
 *  Created on: Mar 3, 2021
 *      Author: armin
 */


#include <op_planner/OpenDriveMapLoader.h>
#include <op_planner/PlanningHelpers.h>
#include <op_planner/opendrive/geometry/GeometryGenerator.hpp>
#include <op_planner/opendrive/parser/OpenDriveParser.hpp>

namespace PlannerHNS {

OpenDriveMapLoader::OpenDriveMapLoader(int map_version) : _map_version(map_version)
{
	_pMap = nullptr;
	_bLaneStitch = false;
	_waypointCounter = 1;
}

OpenDriveMapLoader::~OpenDriveMapLoader()
{
}

void OpenDriveMapLoader::LoadXODR(std::string const &file, RoadNetwork& map)
{
	// std::cout << "***** Called LoadXODR ****" << std::endl;
    opendrive::OpenDriveData odr;
    bool bSuccess =  opendrive::parser::OpenDriveParser::Parse(file.c_str(), odr, opendrive::parser::XmlInputType::FILE);

    // check if the file was loaded successfully
	if(!bSuccess)
	{
		ROS_FATAL("Unable to load Map file!");
		return;
	}

	_pMap = &map;

	// std::cout << " >> Reading Data from OpenDrive map file ... " << std::endl;

	// std::cout << " >> Load Lanes from OpenDrive file .. " << std::endl;
	ROS_INFO(">> Load Lanes from OpenDrive file ... ");
	std::vector<Lane> laneLinksList = GetLanesList(&odr);
	ROS_INFO(">> %d Lanes loaded!", laneLinksList.size());
	MappingHelpers::FixTwoPointsLanes(laneLinksList);

	map.roadSegments.clear();
	//map.roadSegments = GetRoadSegmentsList(&odr);
	//map.roadSegments.at(0).id = 0;

	PlannerHNS::RoadSegment rS;
	rS.id = 0;
	map.roadSegments.push_back(rS);

	//Fill the relations
	for(unsigned int j=0; j < laneLinksList.size(); j++)
	{
		//PlanningHelpers::CalcAngleAndCost(laneLinksList.at(j).points);
		// std::cout << " >> Added pRoad to Lane: " << laneLinksList.at(j).id << std::endl;
		laneLinksList.at(j).pRoad = &map.roadSegments.at(0);
		map.roadSegments.at(0).Lanes.push_back(laneLinksList.at(j));
	}

	ROS_INFO(">> Load Stop Lines from OpenDrive file ... ");
	std::vector<StopLine> stopLines = GetStopLinesList(&odr);

	ROS_INFO(">> Load Traffic Lights from OpenDrive file ... ");
	std::vector<TrafficLight> trafficLights = GetTrafficLightsList(&odr);

	ROS_INFO(">> Load Traffic Signs from OpenDrive file ... ");
	// std::cout << " >> Load Traffic Lights from OpenDrive file ... " << std::endl;
	std::vector<TrafficSign> trafficSigns = GetTrafficSignsList(&odr);

	ROS_INFO(">> Load Curbs from OpenDrive file ... ");
	// std::cout << " >> Load Curbs from OpenDrive file .. " << std::endl;
	std::vector<Curb> curbs = GetCurbsList(&odr);

	ROS_INFO(">> Load Boundaries from OpenDrive file ... ");
	std::vector<Boundary> boundaries = GetBoundariesList(&odr);

	std::cout << " >> Link lanes and waypoints with pointers ... " << std::endl;
	//Link Lanes by pointers
	MappingHelpers::LinkLanesPointers(map);

	//Link waypoints by pointers
	std::cout << " >> Link missing branches and waypoints... " << std::endl;
	MappingHelpers::LinkMissingBranchingWayPointsV2(map);

	//Link lanechange waypoints by pointers
	std::cout << " >> Link Lane change semantics ... " << std::endl;
	MappingHelpers::LinkLaneChangeWaypointsPointers(map);

	if(_bLaneStitch && map.roadSegments.size() > 0)
	{
		MappingHelpers::StitchLanes(map.roadSegments.at(0).Lanes);
	}

	// map.stopLines.clear();
	map.stopLines = stopLines;

	// map.trafficLights.clear();
	map.trafficLights = trafficLights;

	map.curbs.clear();
	map.curbs = curbs;

	map.boundaries = boundaries;

	// std::cout << " >> Link Boundaries and Waypoints ... " << std::endl;
	MappingHelpers::ConnectBoundariesToWayPoints(map);
	MappingHelpers::LinkBoundariesToWayPoints(map);

	MappingHelpers::LinkTrafficLightsIntoGroups(map);
	MappingHelpers::ConnectTrafficLightsAndStopLines(map);
	
	//Link waypoints && StopLines
	// std::cout << " >> Link Stop lines and Traffic lights ... " << std::endl;
	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i = 0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			for(unsigned int itl = 0; itl < map.trafficLights.size(); itl++)
			{	
				if(map.trafficLights.at(itl).CheckLane(map.roadSegments.at(rs).Lanes.at(i).id))
				{
					map.trafficLights.at(itl).pLanes.push_back(&map.roadSegments.at(rs).Lanes.at(i));
					map.roadSegments.at(rs).Lanes.at(i).trafficlights.push_back(map.trafficLights.at(itl));
					// std::cout << " >> Added Traffic Light " << map.trafficLights.at(itl).id << " to Lane." << std::endl;
				}
			}

			for(unsigned int isl = 0; isl < map.stopLines.size(); isl++)
			{
				if(map.stopLines.at(isl).laneId == map.roadSegments.at(rs).Lanes.at(i).id)
				{
					map.stopLines.at(isl).pLane = &map.roadSegments.at(rs).Lanes.at(i);
					map.roadSegments.at(rs).Lanes.at(i).stopLines.push_back(map.stopLines.at(isl));

					// TODO: Figure out how the waypoints are matched. Issue due to lanes which are super close to each other. 
					WayPoint wp((map.stopLines.at(isl).points.at(0).pos.x+map.stopLines.at(isl).points.at(1).pos.x)/2.0, (map.stopLines.at(isl).points.at(0).pos.y+map.stopLines.at(isl).points.at(1).pos.y)/2.0, (map.stopLines.at(isl).points.at(0).pos.z+map.stopLines.at(isl).points.at(1).pos.z)/2.0, (map.stopLines.at(isl).points.at(0).pos.a+map.stopLines.at(isl).points.at(1).pos.a)/2.0);
					// WayPoint wp( map.stopLines.at(isl).points.at(0).pos.x, map.stopLines.at(isl).points.at(0).pos.y, map.stopLines.at(isl).points.at(0).pos.z, map.stopLines.at(isl).points.at(0).pos.a );
					map.roadSegments.at(rs).Lanes.at(i).points.at(PlanningHelpers::GetClosestNextPointIndexFast(map.roadSegments.at(rs).Lanes.at(i).points, wp)).stopLineID = map.stopLines.at(isl).id;
					// std::cout << " >> Added Stop Line " << map.stopLines.at(isl).id << " to Waypoint." << std::endl;
				}
			}
		}
	}

	// std::cout << " >> Find Max IDs ... " << std::endl;
	MappingHelpers::GetMapMaxIds(map);

	// std::cout << "Map loaded from OpenDrive File with (" << laneLinksList.size()  << ") lanes, First Point ( " << MappingHelpers::GetFirstWaypoint(map).pos.ToString() << ")"<< std::endl;

	_pMap = nullptr;
}

std::vector<Lane> OpenDriveMapLoader::GetLanesList(const opendrive::OpenDriveData* odr)
{
	std::vector<Lane> lList;
	// first iteration connect everyting based on ids
	for(const opendrive::RoadInformation &road : odr->roads)
    {
		for (int i = 0; i < road.lanes.lane_sections.size(); i++) {
			double endPosition = 0; 
			if(i < road.lanes.lane_sections.size()-1)
			{
				endPosition = road.lanes.lane_sections[i+1].start_position;
			}
			else
			{
				endPosition = road.attributes.length;
			}
			for(const opendrive::LaneInfo &laneInfoRight : road.lanes.lane_sections[i].right)
			{
				if(	laneInfoRight.attributes.type == opendrive::LaneType::Driving )
				{
					Lane l;
					l = GetLaneInfo(odr, &road, i, &laneInfoRight, road.lanes.lane_sections[i].start_position, endPosition);
					lList.push_back(l);
				}
			}
			for(const opendrive::LaneInfo &laneInfoLeft : road.lanes.lane_sections[i].left)
			{
				if(	laneInfoLeft.attributes.type == opendrive::LaneType::Driving )
				{
					Lane l;
					l = GetLaneInfo(odr, &road, i, &laneInfoLeft, road.lanes.lane_sections[i].start_position, endPosition);
					std::reverse(l.points.begin(), l.points.end());
					lList.push_back(l);
				}
			}
		}
	}
	return lList;
}

std::vector<RoadSegment> OpenDriveMapLoader::GetRoadSegmentsList(const opendrive::OpenDriveData* odr)
{
	std::vector<RoadSegment> rlList;

	if(odr->roads.size() < 1)
		return rlList;

	for(const opendrive::RoadInformation &road : odr->roads)
    {
		RoadSegment rl;
		rl.id = road.attributes.id;
		rlList.push_back(rl);
	}

	return rlList;
}

Lane OpenDriveMapLoader::GetLaneInfo(	const opendrive::OpenDriveData* odr, 
										const opendrive::RoadInformation* road,
										uint8_t laneSectionId, 
										const opendrive::LaneInfo* laneInfo, 
										double startPosition, 
										double endPosition)
{
	Lane ll;
	OpenDriveMapLoader::openDriveLaneId id;
	id.roadId = road->attributes.id;
	id.laneSectionId = laneSectionId;
	id.laneId = laneInfo->attributes.id;
	ll.id = openDriveIDsToInt(id);
	
	// add all lanes to the same road segment
	ll.roadId = 0;
	ll.num = -1;
	ll.type = PlannerHNS::NORMAL_LANE;
	ll.length = endPosition - startPosition;
	if(laneInfo->lane_width.size() > 0)
		ll.width = laneInfo->lane_width[0].a;
	if(road->attributes.speed.size() > 0)
		ll.speed = road->attributes.speed[0].max;
	ll.fromIds 	= GetFromIDs(odr, road, laneInfo);
	ll.toIds 	= GetToIDs(odr, road, laneSectionId, laneInfo);

	// Get Center Lane Data based on OpenDrive Road
	ll.points = GetLaneData(odr, road, laneInfo, startPosition, endPosition, 0.5);
	return ll;
}

// Get IDs of the Predecessing Roads
std::vector<int> OpenDriveMapLoader::GetFromIDs(const opendrive::OpenDriveData* odr, const opendrive::RoadInformation* road, const opendrive::LaneInfo* laneInfo)
{
	std::vector<int> fromIds;

	// check right side element
	// check if a predecessor exists
	if(road->road_link.predecessor && laneInfo->attributes.id < 0)
	{
		//predecessor type is a road
		if(road->road_link.predecessor->element_type == opendrive::ElementType::Road)
		{
			//check right side lanes with contact point end
			if(road->road_link.predecessor->contact_point == opendrive::ContactPoint::End)
			{
				OpenDriveMapLoader::openDriveLaneId id;
				id.roadId = road->road_link.predecessor->id;
				id.laneSectionId = 0;
				id.laneId = laneInfo->attributes.id;
				fromIds.push_back(openDriveIDsToInt(id));
			}
		}

		// no need for checking from Junctions since only incoming Roads are linked
	}

	//check left side element
	//check if a successor exists
	if(road->road_link.successor && laneInfo->attributes.id > 0)
	{
		//checking for roads
		if(road->road_link.successor->element_type == opendrive::ElementType::Road)
		{
			// check left side lanes with contact point end
			if(road->road_link.successor->contact_point == opendrive::ContactPoint::End)
			{
				OpenDriveMapLoader::openDriveLaneId id;
				id.roadId = road->road_link.successor->id;
				id.laneSectionId = 0;
				id.laneId = laneInfo->attributes.id;
				fromIds.push_back(openDriveIDsToInt(id));
			}
				
			// check left side lanes with contact point start
			if(road->road_link.successor->contact_point == opendrive::ContactPoint::Start)
			{
				OpenDriveMapLoader::openDriveLaneId id;
				id.roadId = road->road_link.successor->id;
				id.laneSectionId = 0;
				id.laneId = laneInfo->attributes.id;
				fromIds.push_back(openDriveIDsToInt(id));
			}
		}

		// no need for checking from Junctions since only incoming Roads are linked
	}

	return fromIds;
}

// Get IDs of the Successing Roads
std::vector<int> OpenDriveMapLoader::GetToIDs(const opendrive::OpenDriveData* odr, const opendrive::RoadInformation* road, uint8_t laneSectionId, const opendrive::LaneInfo* laneInfo)
{
	std::vector<int> toIds;

	//first check if there are other laneSections after the current section
	bool found_succeeding_lane_section = false;
	for(const opendrive::RoadInformation &r : odr->roads)
	{
		// connect lane sections on the right side
		if( r.attributes.id == road->attributes.id && laneInfo->attributes.id < 0)
		{
			if(road->lanes.lane_sections.size()-1 > laneSectionId)
			{
				// there are more sections after this laneSection
				for (int i = 0; i < road->lanes.lane_sections.size(); i++) {
					if(i > laneSectionId)
					{
						for(const opendrive::LaneInfo &laneInfoRight : road->lanes.lane_sections.at(i).right)
						{
							if(	laneInfoRight.attributes.type == opendrive::LaneType::Driving )
							{
								found_succeeding_lane_section = true;
								OpenDriveMapLoader::openDriveLaneId id;
								id.roadId = road->attributes.id;
								id.laneSectionId = i;
								id.laneId = laneInfoRight.attributes.id;
								toIds.push_back(openDriveIDsToInt(id));
							}
						}
						break;
					}
				}
			}
			break;	
		}

		// connect lane sections on the left side
		if( r.attributes.id == road->attributes.id && laneInfo->attributes.id > 0)
		{
			if(laneSectionId > 0)
			{
				// there are more sections after this laneSection
				for (int i = road->lanes.lane_sections.size()-1; i >=0; i--) {
					if(i < laneSectionId)
					{
						for(const opendrive::LaneInfo &laneInfoLeft : road->lanes.lane_sections.at(i).left)
						{
							if(	laneInfoLeft.attributes.type == opendrive::LaneType::Driving )
							{
								found_succeeding_lane_section = true;
								OpenDriveMapLoader::openDriveLaneId id;
								id.roadId = road->attributes.id;
								id.laneSectionId = i;
								id.laneId = laneInfoLeft.attributes.id;
								toIds.push_back(openDriveIDsToInt(id));
							}
						}
						break;
					}
				}
			}
			break;	
		}
	}

	// check right side element
	// also the laneSectionId has to be the last Sectionf for the successor evalutation
	// check if a successor exists
	if(road->road_link.successor && laneInfo->attributes.id < 0 && laneSectionId == road->lanes.lane_sections.size()-1)
	{
		//successor type is road
		if(road->road_link.successor->element_type == opendrive::ElementType::Road)
		{
			for(const opendrive::LaneInfo &laneInfoRight : road->lanes.lane_sections[laneSectionId].right)
			{
				if(laneInfoRight.attributes.id == laneInfo->attributes.id)
				{

					OpenDriveMapLoader::openDriveLaneId id;
					id.roadId = road->road_link.successor->id;
					id.laneId = laneInfoRight.link->successor_id;
					id.laneSectionId = 0;
					if(id.laneId > 0)
					{
						for(const opendrive::RoadInformation &r : odr->roads)
						{
							if( r.attributes.id == id.roadId)
								id.laneSectionId = r.lanes.lane_sections.size()-1;
						}
					}
					id.laneId = laneInfoRight.link->successor_id;
					toIds.push_back(openDriveIDsToInt(id));
				}
			}
		}

		//successor type is a junction
		if(road->road_link.successor->element_type == opendrive::ElementType::Junction)
		{
			// loop trough all junctions in odr map
			for(const opendrive::Junction &junction : odr->junctions)
			{
				if(junction.attributes.id == road->road_link.successor->id)
				{
					for(const opendrive::JunctionConnection &connection: junction.connections)
					{
						if(connection.attributes.incoming_road == road->attributes.id && connection.links.size() > 0)
						{
							for(unsigned int i = 0; i < connection.links.size(); i++)
							{
								if(laneInfo->attributes.id == connection.links.at(i).from)
								{
									OpenDriveMapLoader::openDriveLaneId id;
									id.roadId = connection.attributes.connecting_road;
									id.laneId = connection.links.at(i).to;
									id.laneSectionId = 0;
									if(id.laneId > 0)
									{
										for(const opendrive::RoadInformation &r : odr->roads)
										{
											if( r.attributes.id == id.roadId)
												id.laneSectionId = r.lanes.lane_sections.size()-1;
										}
									}
									
									toIds.push_back(openDriveIDsToInt(id));
								}
							}
						}
					}
				}
			}
		}
	}

	// check left side element
	// since left side elements are opposite to the s-coordinate frame we have to look for predecessors not successors
	// also the laneSectionId has to be zero for the successor evalutation
	// check if a predecessor exists
	if(road->road_link.predecessor && laneInfo->attributes.id > 0 && laneSectionId == 0)
	{
		//predecessor type is a road
		if(road->road_link.predecessor->element_type == opendrive::ElementType::Road)
		{
			for(const opendrive::LaneInfo &laneInfoLeft : road->lanes.lane_sections[laneSectionId].left)
			{
				if(laneInfoLeft.attributes.id == laneInfo->attributes.id)
				{
					OpenDriveMapLoader::openDriveLaneId id;
					id.roadId = road->road_link.predecessor->id;
					id.laneId = laneInfoLeft.link->predecessor_id;
					id.laneSectionId = 0;
					if(id.laneId > 0)
					{
						for(const opendrive::RoadInformation &r : odr->roads)
						{
							if( r.attributes.id == id.roadId)
								id.laneSectionId = r.lanes.lane_sections.size()-1;
						}
					}
					
					toIds.push_back(openDriveIDsToInt(id));
				}
			}
		}

		//predecessor type is a junction
		if(road->road_link.predecessor->element_type == opendrive::ElementType::Junction)
		{
			// loop trough all junctions in odr map
			for(const opendrive::Junction &junction : odr->junctions)
			{
				if(junction.attributes.id == road->road_link.predecessor->id)
				{
					for(const opendrive::JunctionConnection &connection: junction.connections)
					{
						if(connection.attributes.incoming_road == road->attributes.id && connection.links.size() > 0)
						{
							for(unsigned int i = 0; i < connection.links.size(); i++)
							{
								if(laneInfo->attributes.id == connection.links.at(i).from)
								{
									OpenDriveMapLoader::openDriveLaneId id;
									id.roadId = connection.attributes.connecting_road;
									id.laneId = connection.links.at(i).to;
									id.laneSectionId = 0;
									if(id.laneId > 0)
									{
										for(const opendrive::RoadInformation &r : odr->roads)
										{
											if( r.attributes.id == id.roadId)
												id.laneSectionId = r.lanes.lane_sections.size()-1;
										}
									}
									toIds.push_back(openDriveIDsToInt(id));
								}
							}
						}
					}
				}
			}
		}
	}

	return toIds;
}

std::vector<WayPoint> OpenDriveMapLoader::GetLaneData(	const opendrive::OpenDriveData* odr, 
																const opendrive::RoadInformation* road, 
																const opendrive::LaneInfo* laneInfo,
																double startPosition,
																double endPosition,
																double factor)
{
	std::vector<WayPoint> gps_points;
	OpenDriveMapLoader::openDriveLaneId id;
	id.roadId = road->attributes.id;
	id.laneSectionId = 0;
	id.laneId = laneInfo->attributes.id;
	int laneId = openDriveIDsToInt(id);

	std::vector<opendrive::LaneWidth> lW = GetLaneWidths(odr, road, laneInfo, startPosition, factor);

	// iterate trough all geometries defining the road
	for(const std::unique_ptr<opendrive::GeometryAttributes> &geometry: road->geometry_attributes)
	{
		int sideId = laneInfo->attributes.id;
		try
		{
			switch (geometry->type)
			{
				case opendrive::GeometryType::ARC:
				{
					auto arc = static_cast<opendrive::GeometryAttributesArc *>(geometry.get());
					for(double sArc = startPosition; sArc < arc->length; sArc += _resolution)
					{							
						if(sArc > arc->length)
							sArc = arc->length;
						if(startPosition <= sArc && sArc <= endPosition)
						{
							PlannerHNS::WayPoint p = GeneratePointFromArc( arc, road, lW, laneId, _waypointCounter, sideId, arc->start_position, sArc );
							gps_points.push_back(p);
							_waypointCounter++;
						}

					}
					// add one last waypoint at arc->length
					if(startPosition <= arc->length && arc->length <= endPosition)
					{
						PlannerHNS::WayPoint p = GeneratePointFromArc( arc, road, lW, laneId, _waypointCounter, sideId, arc->start_position, arc->length );
						gps_points.push_back(p);
						_waypointCounter++;
					}
					
					break;
				}
				break;
				case opendrive::GeometryType::LINE:
				{
					auto line = static_cast<opendrive::GeometryAttributesLine *>(geometry.get());
					for(double sLine = startPosition; sLine < line->length; sLine += _resolution)
					{
						if(sLine > line->length)
							sLine = line->length;
						if(startPosition <= sLine && sLine <= endPosition)
						{
							PlannerHNS::WayPoint p = GeneratePointFromLine( line, road, lW, laneId, _waypointCounter, sideId, line->start_position, sLine );
							gps_points.push_back(p);
							_waypointCounter++;
						}
					}
					// add one last waypoint at line->length
					if(startPosition <= line->length && line->length <= endPosition)
					{
						PlannerHNS::WayPoint p = GeneratePointFromLine( line, road, lW, laneId, _waypointCounter, sideId, line->start_position, line->length );
						gps_points.push_back(p);
						_waypointCounter++;
					}
					break;
				}
				break;
				case opendrive::GeometryType::SPIRAL:
				{
					ROS_FATAL(">> SPIRAL Geometries are not covered by GetCenterLaneData!");
					break;
				}
				break;
				case opendrive::GeometryType::POLY3:
				{
					ROS_FATAL(">> POLY3 Geometries are not covered by GetCenterLaneData!");
					break;
				}
				break;
				case opendrive::GeometryType::PARAMPOLY3:
				{
					ROS_FATAL(">> PARAMPOLY3 Geometries are not covered by GetCenterLaneData!");
					break;
				}
				break;
				default:
				break;
			}
		}
		catch (...)
		{
			ROS_FATAL(">> Geometries are not covered by GetCenterLaneData!");
			continue;
		}

	}

	return gps_points;
}


PlannerHNS::WayPoint OpenDriveMapLoader::GeneratePointFromLine(	const opendrive::GeometryAttributesLine *line, 
																const opendrive::RoadInformation* road,
																const std::vector<opendrive::LaneWidth> width, 
																int laneId, 
																unsigned int waypointId, 
																int sideId, 
																double sOffset, 
																double ds)
{
	PlannerHNS::WayPoint p;
	p.laneId = laneId;
	p.id = waypointId;
	p.bDir = PlannerHNS::FORWARD_DIR;

	p.width = -1;
	double sWidth = ds;
	
	std::vector<opendrive::LaneOffset> laneOffset = road->lanes.lane_offset;

	// calculate the road width at the given ds
	if(width.size() == 1)
	{
		p.width = width[0].a + (ds - width[0].soffset) * width[0].b + pow((ds - width[0].soffset), 2.0) * width[0].c + pow((ds - width[0].soffset), 3.0) * width[0].d;
	}
	else if (width.size() > 1)
	{
		for(int i = 0; i < width.size()-1; i++)
		{	
			if(sWidth >= width[i].soffset && sWidth < width[i+1].soffset)
			{
				p.width = width[i].a + (ds - width[i].soffset) * width[i].b + pow((ds - width[i].soffset), 2.0) * width[i].c + pow((ds - width[i].soffset), 3.0) * width[i].d;
				break;
			}
		}
		if(sWidth >= width[width.size()-1].soffset)
		{
			p.width = width[width.size()-1].a + (ds - width[width.size()-1].soffset) * width[width.size()-1].b + pow((ds - width[width.size()-1].soffset), 2.0) * width[width.size()-1].c + pow((ds - width[width.size()-1].soffset), 3.0) * width[width.size()-1].d;
		}
		if(p.width == -1)
		{
			ROS_FATAL(">> No sOffset segment found for Line Geometry in lane %d!", p.laneId);
			ROS_FATAL(">> sWidth: %f, ds: %f, width.size(): %d, width[0].soffset: %f, width[width.size()-1].soffset: %f!", sWidth, ds, width.size(), width[0].soffset, width[width.size()-1].soffset);
		}
	}
	else
	{
		ROS_FATAL(">> No width found!");
	}

	double temp_width = -1;

	// calculate the road laneOffset at the given ds
	if(laneOffset.size() == 1)
	{
		temp_width = laneOffset[0].a + ds * laneOffset[0].b + pow(ds, 2.0) * laneOffset[0].c + pow(ds, 3.0) * laneOffset[0].d;
	}
	else if (laneOffset.size() > 1)
	{
		for(int i = 0; i < laneOffset.size()-1; i++)
		{
			if(sWidth >= laneOffset[i].s && sWidth < laneOffset[i+1].s)
			{
				temp_width = laneOffset[i].a + ds * laneOffset[i].b + pow(ds, 2.0) * laneOffset[i].c + pow(ds, 3.0) * laneOffset[i].d;
				break;
			}
		}
		if(sWidth >= laneOffset.back().s)
		{
			temp_width = laneOffset.back().a + ds * laneOffset.back().b + pow(ds, 2.0) * laneOffset.back().c + pow(ds, 3.0) * laneOffset.back().d;
		}
		if(temp_width == -1)
		{
			ROS_FATAL(">> No s segment found for Line Geometry in lane %d!", p.laneId);
			ROS_FATAL(">> sWidth: %f, ds: %f, laneOffset.size(): %d, laneOffset[0].s: %f, laneOffset.back().s: %f!", sWidth, ds, laneOffset.size(), laneOffset.back().s, laneOffset.back().s);
		}
	}
	else
	{
		ROS_FATAL(">> No laneOffset found!");
	}
	if((temp_width < 0.0001 && temp_width > -0.0001) == false)
		p.width -= temp_width;
	
	// change heading offset to negative
	if(sideId <= -1)
		sideId = -1;
	// change heading offset to positive
	if(sideId >= 1)
		sideId = 1;

	// calculate the heading at the given ds
	tf2::Quaternion q;
	double heading = line->heading;
	if ( sideId > 0)
		heading += M_PI;
	q.setRPY( 0, 0, heading );
	p.rot.x = q[0];
	p.rot.y = q[1];
	p.rot.z = q[2];
	p.rot.w = q[3];

	// line defined by s reference frame
	p.pos.x = line->start_position_x + cos(line->heading) * ds + cos(line->heading + sideId * M_PI_2) * p.width;
	p.pos.y = line->start_position_y + sin(line->heading) * ds + sin(line->heading + sideId * M_PI_2) * p.width;
	p.pos.z = 0;

	p.pos.a = heading;
	p.cost = 0.0;

	return p;
}

PlannerHNS::WayPoint OpenDriveMapLoader::GeneratePointFromArc(	const opendrive::GeometryAttributesArc *arc, 
																const opendrive::RoadInformation* road, 
																const std::vector<opendrive::LaneWidth> width, 
																int laneId, 
																unsigned int waypointId, 
																int sideId, 
																double sOffset, 
																double ds)
{
	PlannerHNS::WayPoint p;
	p.laneId = laneId;
	p.id = waypointId;
	p.bDir = PlannerHNS::FORWARD_DIR;
	p.width = -1;

	std::vector<opendrive::LaneOffset> laneOffset = road->lanes.lane_offset;
	double sWidth = ds;
	// if(sOffset != 0)
	// 	ROS_INFO("Generating Arc for Lane with sOffset of: %f", sOffset);
	// calculate the road width at the given ds
	if(width.size() == 1)
	{
		p.width = width[0].a + (ds - width[0].soffset) * width[0].b + pow((ds - width[0].soffset), 2.0) * width[0].c + pow((ds - width[0].soffset), 3.0) * width[0].d;
	}
	else if (width.size() > 1)
	{
		for(int i = 0; i < width.size()-1; i++)
		{	
			if(sWidth >= width[i].soffset && sWidth < width[i+1].soffset)
			{
				p.width = width[i].a + (ds - width[i].soffset) * width[i].b + pow((ds - width[i].soffset), 2.0) * width[i].c + pow((ds - width[i].soffset), 3.0) * width[i].d;
				break;
			}
		}
		if(sWidth >= width[width.size()-1].soffset)
		{
			p.width = width[width.size()-1].a + (ds - width[width.size()-1].soffset) * width[width.size()-1].b + pow((ds - width[width.size()-1].soffset), 2.0) * width[width.size()-1].c + pow((ds - width[width.size()-1].soffset), 3.0) * width[width.size()-1].d;
		}
		if(p.width == -1)
		{
			ROS_FATAL(">> No sOffset segment found for Arc Geometry in lane %d!", p.laneId);
			ROS_FATAL(">> sWidth: %f, ds: %f, width.size(): %d, width[0].soffset: %f, width[width.size()-1].soffset: %f!", sWidth, ds, width.size(), width[0].soffset, width[width.size()-1].soffset);
		}
	}
	else
	{
		ROS_FATAL(">> No width found!");
	}

	double temp_width = -1;

	// calculate the road laneOffset at the given ds
	if(laneOffset.size() == 1)
	{
		temp_width = laneOffset[0].a + ds * laneOffset[0].b + pow(ds, 2.0) * laneOffset[0].c + pow(ds, 3.0) * laneOffset[0].d;
	}
	else if (laneOffset.size() > 1)
	{
		int last_index = laneOffset.size()-1;
		for(int i = 0; i < last_index; i++)
		{
			if(sWidth >= laneOffset[i].s && sWidth < laneOffset[i+1].s)
			{
				temp_width = laneOffset[i].a + ds * laneOffset[i].b + pow(ds, 2.0) * laneOffset[i].c + pow(ds, 3.0) * laneOffset[i].d;
				break;
			}
		}
		if(sWidth >= laneOffset[last_index].s)
		{
			temp_width = laneOffset[last_index].a + ds * laneOffset[last_index].b + pow(ds, 2.0) * laneOffset[last_index].c + pow(ds, 3.0) * laneOffset[last_index].d;
		}
		if(temp_width == -1)
		{
			ROS_FATAL(">> No s segment found for Arc Geometry in lane %d!", p.laneId);
			ROS_FATAL(">> sWidth: %f, ds: %f, laneOffset.size(): %d, laneOffset[0].s: %f, laneOffset[last_index].s: %f!", sWidth, ds, laneOffset.size(), laneOffset[0].s, laneOffset[last_index].s);
		}
	}
	else
	{
		ROS_FATAL(">> No laneOffset found!");
	}

	p.width -= temp_width;

	// calculate arc parameters
	double R = 1 / arc->curvature;
	double circumference = 2.0 * R * M_PI;
	double alpha_ds = (ds / circumference) * 2.0 * M_PI;
	double angularOffset = M_PI_2;
	int offsetSign = 0.0;

	// Left Side and Right curve
	if( sideId > 0 && arc->curvature < 0 )
	{
		offsetSign = -1;
	}
	// Left Side and Left curve
	if ( sideId > 0 && arc->curvature >= 0 )
	{
		offsetSign = -1;
	}
	// Right Side and Right curve
	if ( sideId < 0 && arc->curvature < 0 )
	{
		offsetSign = 1;
	}
	// Right Side and Left curve
	if ( sideId < 0 && arc->curvature >= 0 )
	{
		offsetSign = 1;
	}

	double heading = arc->heading + alpha_ds;
	if ( sideId > 0)
		heading += M_PI;
	
	double circle_center_x = arc->start_position_x + cos(arc->heading + angularOffset) * R;
	double circle_center_y = arc->start_position_y + sin(arc->heading + angularOffset) * R;	

	double wOffset = offsetSign * p.width;
	double dx = (R + wOffset) * cos(M_PI + arc->heading + angularOffset + alpha_ds);
	double dy = (R + wOffset) * sin(M_PI + arc->heading + angularOffset + alpha_ds);

	// calculate the heading at the given ds
	tf2::Quaternion q;
	
	q.setRPY( 0, 0, heading);
	p.rot.x = q[0];
	p.rot.y = q[1];
	p.rot.z = q[2];
	p.rot.w = q[3];
	
	p.pos.x = circle_center_x + dx;
	p.pos.y = circle_center_y + dy;
	p.pos.z = 0;
	p.pos.a = heading;
	p.cost = 0.0;

	return p;
}

// Create Stop Lines for each signal with name == "StopLine" based on s_position, lane width and geometries
std::vector<StopLine> OpenDriveMapLoader::GetStopLinesList(const opendrive::OpenDriveData* odr)
{
	std::vector<StopLine> slList;
	// first iteration connect everyting based on ids
	for(const opendrive::RoadInformation &road : odr->roads)
    {
		if(road.traffic_signal_references.size() > 0)
		{
			// get the current road's lane ids
			int laneId = 0;
			std::vector<opendrive::LaneWidth> width;
			for(const opendrive::LaneSection &laneSection : road.lanes.lane_sections)
			{
				for(const opendrive::LaneInfo &laneInfoRight : laneSection.right)
				{
					if(	laneInfoRight.attributes.type == opendrive::LaneType::Driving
						|| laneInfoRight.attributes.type == opendrive::LaneType::Shoulder)
					{
						laneId = laneInfoRight.attributes.id;
						width = GetLaneWidths(odr, &road, &laneInfoRight, 0, 0.5);
						break;
					}
				}
				for(const opendrive::LaneInfo &laneInfoLeft : laneSection.left)
				{
					if(	laneInfoLeft.attributes.type == opendrive::LaneType::Driving
						|| laneInfoLeft.attributes.type == opendrive::LaneType::Shoulder)
					{
						laneId = laneInfoLeft.attributes.id;
						width = GetLaneWidths(odr, &road, &laneInfoLeft, 0, 0.5);
						break;
					}
				}
			}

			if(laneId != 0)
			{
				for(int i = 0; i < road.traffic_signal_references.size(); i++)
				{
					StopLine sl;
					//get base point of Stop Line
					double s_pos = road.traffic_signal_references.at(i).start_position;
					double t_pos = road.traffic_signal_references.at(i).track_position;

					int sideId = 0;
					if(laneId > 0)
					{
						sideId = 1;
					}
					if(laneId < 0)
					{
						sideId = -1;
					}
					OpenDriveMapLoader::openDriveLaneId id;
					id.roadId = road.traffic_signal_references.at(i).id;
					id.laneSectionId = 0;
					id.laneId = 0;
					int tl_id = openDriveIDsToInt(id);
					sl.id = tl_id;
					sl.lightIds.push_back(sl.id);

					// flip the s_pos starting point since the geometry was defined in the other direction
					if(sideId * t_pos < 0)
						s_pos = road.attributes.length - s_pos;

					PlannerHNS::WayPoint pRef;
					// iterate trough all geometries defining the road to get the geometry at s_pos
					unsigned int geoIndex = 0;
					for(unsigned int i = 0; i < road.geometry_attributes.size(); i ++)
					{
						if(i < road.geometry_attributes.size()-1)
						{
							if(road.geometry_attributes.at(i)->start_position < s_pos && road.geometry_attributes.at(i + 1)->start_position > s_pos)
								geoIndex = i;
							
						}
						if(i == road.geometry_attributes.size()-1)
						{
							if(road.geometry_attributes.at(i)->start_position < s_pos)
								geoIndex = i;
						}
					}

					const std::unique_ptr<opendrive::GeometryAttributes> &geometry = road.geometry_attributes.at(geoIndex);

					// get the gps position of the stop Line reference
					try
					{
						switch (geometry->type)
						{
							case opendrive::GeometryType::ARC:
							{
								auto arc = static_cast<opendrive::GeometryAttributesArc *>(geometry.get());
								double sArc = s_pos - arc->start_position;

								if(sArc > arc->length)
									sArc = arc->length;
								if(sArc < 0)
									sArc = 0;
								pRef = GeneratePointFromArc( arc, &road, width, sl.laneId, 0, sideId, arc->start_position, sArc);
								break;
							}
							break;
							case opendrive::GeometryType::LINE:
							{
								auto line = static_cast<opendrive::GeometryAttributesLine *>(geometry.get());
								double sLine = s_pos - line->start_position;
								if(sLine > line->length)
									sLine = line->length;
								if(sLine < 0)
									sLine = 0;
								pRef = GeneratePointFromLine( line, &road, width, sl.laneId, 0, sideId, line->start_position, sLine );
								break;
							}
							break;
							case opendrive::GeometryType::SPIRAL:
							{
								ROS_FATAL(">> SPIRAL Geometries are not covered by GetCenterLaneData!");
								break;
							}
							break;
							case opendrive::GeometryType::POLY3:
							{
								ROS_FATAL(">> POLY3 Geometries are not covered by GetCenterLaneData!");
								break;
							}
							break;
							case opendrive::GeometryType::PARAMPOLY3:
							{
								ROS_FATAL(">> PARAMPOLY3 Geometries are not covered by GetCenterLaneData!");
								break;
							}
							break;
							default:
							break;
						}
					}
					catch (...)
					{
						ROS_FATAL(">> In GetStopLinesList: Geometries are not covered by GetCenterLaneData!");
					}
		
					// create Points
					sl.points.clear();
					int slLength = 2;

					for(int i = 0; i < slLength; i++)
					{
						PlannerHNS::WayPoint p;
						p.pos.x = pRef.pos.x + cos(pRef.pos.a + M_PI_2) * pRef.width * (double)(1 - i * slLength);
						p.pos.y = pRef.pos.y + sin(pRef.pos.a + M_PI_2) * pRef.width * (double)(1 - i * slLength);
						p.pos.z = pRef.pos.z;

						sl.points.push_back(p);
					}

					if(laneId !=0){
						OpenDriveMapLoader::openDriveLaneId id;
						id.roadId = road.attributes.id;
						id.laneSectionId = 0;
						id.laneId = laneId;
						sl.laneId = openDriveIDsToInt(id);
					}else{
						ROS_FATAL(">> Did not get a lane Id continuing loop! ");
						continue;
					}
					slList.push_back(sl);
					
				}
			}
			
		}
	}
	return slList;

}

// Create Traffic Light for each signal with type == 1000001
std::vector<TrafficLight> OpenDriveMapLoader::GetTrafficLightsList(const opendrive::OpenDriveData* odr)
{
	std::vector<TrafficLight> tlList;

	// iterate trough roads to identify signals
	for(const opendrive::RoadInformation &road : odr->roads)
    {
		if(road.traffic_signals.size() > 0)
		{
			// check if there is a traffic signal with traffic light 
			for(int i = 0; i < road.traffic_signals.size(); i++)
			{
				if(road.traffic_signals.at(i).type == "1000001")
				{
					
					TrafficLight tl;
					//get base point of Stop Line
					double s_pos = road.traffic_signals.at(i).start_position;
					double t_pos = road.traffic_signals.at(i).track_position;

					OpenDriveMapLoader::openDriveLaneId id;
					id.roadId = road.traffic_signals.at(i).id;
					id.laneSectionId = 0;
					id.laneId = 0;

					tl.id = openDriveIDsToInt(id);

					// iterate trough roads to identify signal references
					for(const opendrive::RoadInformation &road : odr->roads)
					{
						if(road.traffic_signal_references.size() > 0)
						{
							for(int i = 0; i < road.traffic_signal_references.size(); i++)
							{	
								OpenDriveMapLoader::openDriveLaneId ref_id;
								ref_id.roadId = road.traffic_signal_references.at(i).id;
								ref_id.laneSectionId = 0;
								ref_id.laneId = 0;
								if(tl.id == openDriveIDsToInt(ref_id))
								{
									// get the current road's lane id
									int laneId = 0;
									const std::vector<opendrive::LaneWidth>* width;
									for(const opendrive::LaneSection &laneSection : road.lanes.lane_sections)
									{
										for(const opendrive::LaneInfo &laneInfoRight : laneSection.right)
										{
											if(	laneInfoRight.attributes.type == opendrive::LaneType::Driving
												|| laneInfoRight.attributes.type == opendrive::LaneType::Shoulder)
											{
												laneId = laneInfoRight.attributes.id;
												width = &laneInfoRight.lane_width;
												break;
											}
										}
										for(const opendrive::LaneInfo &laneInfoLeft : laneSection.left)
										{
											if(	laneInfoLeft.attributes.type == opendrive::LaneType::Driving
												|| laneInfoLeft.attributes.type == opendrive::LaneType::Shoulder)
											{
												laneId = laneInfoLeft.attributes.id;
												width = &laneInfoLeft.lane_width;
												break;
											}
										}
									}

									if(laneId != 0)
									{	
										OpenDriveMapLoader::openDriveLaneId l_id;
										l_id.roadId = road.attributes.id;
										l_id.laneSectionId = 0;
										l_id.laneId = laneId;
										tl.laneIds.push_back(openDriveIDsToInt(l_id));
										tl.stopLineID = tl.id;

										if(tl.laneIds.size() > 0 && tl.stopLineID != 0)
										{
											;
											// ROS_INFO("Found %d Lanes and Stop Line %d for traffic Light: %d", tl.laneIds.size(), tl.stopLineID, tl.id);
										}else{
											ROS_FATAL("No Lanes and Stop Line found for Traffic Light: %d", tl.id);
										}

									}
								}
							}
						}
					}

					tl.lightType = RED_LIGHT;
					
					tl.vertical_angle = 2 * M_PI;
					tl.horizontal_angle = 2 * M_PI;

					// get the current road's lane id
					int laneId = 0;
					const std::vector<opendrive::LaneWidth>* width;
					for(const opendrive::LaneSection &laneSection : road.lanes.lane_sections)
					{
						for(const opendrive::LaneInfo &laneInfoRight : laneSection.right)
						{
							if(	laneInfoRight.attributes.type == opendrive::LaneType::Driving
								|| laneInfoRight.attributes.type == opendrive::LaneType::Shoulder)
							{
								laneId = laneInfoRight.attributes.id;
								width = &laneInfoRight.lane_width;
								break;
							}
						}
						for(const opendrive::LaneInfo &laneInfoLeft : laneSection.left)
						{
							if(	laneInfoLeft.attributes.type == opendrive::LaneType::Driving
								|| laneInfoLeft.attributes.type == opendrive::LaneType::Shoulder)
							{
								laneId = laneInfoLeft.attributes.id;
								width = &laneInfoLeft.lane_width;
								break;
							}
						}
					}
					// ROS_INFO("Lane ID: %d", laneId);
					if(laneId != 0)
					{
						// ROS_INFO("Creating traffic light: %d", tl.id);
						
						int sideId = 0;
						if(laneId > 0)
						{
							sideId = 1;
						}
						if(laneId < 0)
						{
							sideId = -1;
						}

						// flip the s_pos starting point since the geometry was defined in the other direction
						// if(sideId > 0)
						// 	s_pos = road.attributes.length - s_pos;

						PlannerHNS::WayPoint pRef;
						// iterate trough all geometries defining the road to get the geometry at s_pos
						unsigned int geoIndex = 0;
						for(unsigned int i = 0; i < road.geometry_attributes.size(); i ++)
						{
							if(i < road.geometry_attributes.size()-1)
							{
								if(road.geometry_attributes.at(i)->start_position < s_pos && road.geometry_attributes.at(i + 1)->start_position > s_pos)
									geoIndex = i;
								
							}
							if(i == road.geometry_attributes.size()-1)
							{
								if(road.geometry_attributes.at(i)->start_position < s_pos)
									geoIndex = i;
							}
						}

						const std::unique_ptr<opendrive::GeometryAttributes> &geometry = road.geometry_attributes.at(geoIndex);

						// get the gps position of the traffic Light reference
						try
						{
							switch (geometry->type)
							{
								case opendrive::GeometryType::ARC:
								{
									auto arc = static_cast<opendrive::GeometryAttributesArc *>(geometry.get());
									double sArc = s_pos - arc->start_position;
									if(sArc > arc->length)
										sArc = arc->length;
									pRef = GeneratePointFromArc( arc, &road, *width, laneId, 0, sideId, arc->start_position, sArc);
									break;
								}
								break;
								case opendrive::GeometryType::LINE:
								{
									auto line = static_cast<opendrive::GeometryAttributesLine *>(geometry.get());
									double sLine = s_pos - line->start_position;
									if(sLine > line->length)
										sLine = line->length;
									pRef = GeneratePointFromLine( line, &road, *width, laneId, 0, sideId, line->start_position, sLine );
									break;
								}
								break;
								case opendrive::GeometryType::SPIRAL:
								{
									ROS_FATAL(">> SPIRAL Geometries are not covered by GetCenterLaneData!");
									break;
								}
								break;
								case opendrive::GeometryType::POLY3:
								{
									ROS_FATAL(">> POLY3 Geometries are not covered by GetCenterLaneData!");
									break;
								}
								break;
								case opendrive::GeometryType::PARAMPOLY3:
								{
									ROS_FATAL(">> PARAMPOLY3 Geometries are not covered by GetCenterLaneData!");
									break;
								}
								break;
								default:
								break;
							}
						}
						catch (...)
						{
							ROS_FATAL(">> In GetTrafficLightsList: Geometries are not covered by GetCenterLaneData!");
							continue;
						}

						PlannerHNS::WayPoint p;
						p.pos.x = pRef.pos.x + cos(pRef.pos.a - M_PI_2) * (t_pos); //* sideId);
						p.pos.y = pRef.pos.y + sin(pRef.pos.a - M_PI_2) * (t_pos); //* sideId);
						p.pos.z = road.traffic_signals.at(i).zoffset;

						tl.pose = p;

						tlList.push_back(tl);
					}
				}
			}
		}
	}
	return tlList;
}

// Create Traffic Light for each signal with type == 1000001
std::vector<TrafficSign> OpenDriveMapLoader::GetTrafficSignsList(const opendrive::OpenDriveData* odr)
{
	std::vector<TrafficSign> tsList;
	std::vector<TrafficLight> tlList;

	// iterate trough roads to identify signals
	for(const opendrive::RoadInformation &road : odr->roads)
    {
		if(road.traffic_signals.size() > 0)
		{
			// check if there is a traffic signal with traffic light 
			for(int i = 0; i < road.traffic_signals.size(); i++)
			{
				if(road.traffic_signals.at(i).type == "1000001")
				{
					
					TrafficLight tl;
					//get base point of Stop Line
					double s_pos = road.traffic_signals.at(i).start_position;
					double t_pos = road.traffic_signals.at(i).track_position;

					OpenDriveMapLoader::openDriveLaneId id;
					id.roadId = road.traffic_signals.at(i).id;
					id.laneSectionId = 0;
					id.laneId = 0;

					tl.id = openDriveIDsToInt(id);

					// iterate trough roads to identify signal references
					for(const opendrive::RoadInformation &road : odr->roads)
					{
						if(road.traffic_signal_references.size() > 0)
						{
							for(int i = 0; i < road.traffic_signal_references.size(); i++)
							{	
								OpenDriveMapLoader::openDriveLaneId ref_id;
								ref_id.roadId = road.traffic_signal_references.at(i).id;
								ref_id.laneSectionId = 0;
								ref_id.laneId = 0;
								if(tl.id == openDriveIDsToInt(ref_id))
								{
									// get the current road's lane id
									int laneId = 0;
									const std::vector<opendrive::LaneWidth>* width;
									for(const opendrive::LaneSection &laneSection : road.lanes.lane_sections)
									{
										for(const opendrive::LaneInfo &laneInfoRight : laneSection.right)
										{
											if(	laneInfoRight.attributes.type == opendrive::LaneType::Driving
												|| laneInfoRight.attributes.type == opendrive::LaneType::Shoulder)
											{
												laneId = laneInfoRight.attributes.id;
												width = &laneInfoRight.lane_width;
												break;
											}
										}
										for(const opendrive::LaneInfo &laneInfoLeft : laneSection.left)
										{
											if(	laneInfoLeft.attributes.type == opendrive::LaneType::Driving
												|| laneInfoLeft.attributes.type == opendrive::LaneType::Shoulder)
											{
												laneId = laneInfoLeft.attributes.id;
												width = &laneInfoLeft.lane_width;
												break;
											}
										}
									}

									if(laneId != 0)
									{		
										OpenDriveMapLoader::openDriveLaneId l_id;
										l_id.roadId = road.attributes.id;
										l_id.laneSectionId = 0;
										l_id.laneId = laneId;
										tl.laneIds.push_back(openDriveIDsToInt(l_id));
										tl.stopLineID = tl.id;

										if(tl.laneIds.size() > 0 && tl.stopLineID != 0)
										{
											;
											// ROS_INFO("Found %d Lanes and Stop Line %d for traffic Light: %d", tl.laneIds.size(), tl.stopLineID, tl.id);
										}else{
											ROS_FATAL("No Lanes and Stop Line found for Traffic Light: %d", tl.id);
										}

									}
								}
							}
						}
					}

					tl.lightType = RED_LIGHT;
					
					tl.vertical_angle = 2 * M_PI;
					tl.horizontal_angle = 2 * M_PI;

					// get the current road's lane id
					int laneId = 0;
					const std::vector<opendrive::LaneWidth>* width;
					for(const opendrive::LaneSection &laneSection : road.lanes.lane_sections)
					{
						for(const opendrive::LaneInfo &laneInfoRight : laneSection.right)
						{
							if(	laneInfoRight.attributes.type == opendrive::LaneType::Driving
								|| laneInfoRight.attributes.type == opendrive::LaneType::Shoulder)
							{
								laneId = laneInfoRight.attributes.id;
								width = &laneInfoRight.lane_width;
								break;
							}
						}
						for(const opendrive::LaneInfo &laneInfoLeft : laneSection.left)
						{
							if(	laneInfoLeft.attributes.type == opendrive::LaneType::Driving
								|| laneInfoLeft.attributes.type == opendrive::LaneType::Shoulder)
							{
								laneId = laneInfoLeft.attributes.id;
								width = &laneInfoLeft.lane_width;
								break;
							}
						}
					}
					// ROS_INFO("Lane ID: %d", laneId);
					if(laneId != 0)
					{
						// ROS_INFO("Creating traffic light: %d", tl.id);
						
						int sideId = 0;
						if(laneId > 0)
						{
							sideId = 1;
						}
						if(laneId < 0)
						{
							sideId = -1;
						}

						// flip the s_pos starting point since the geometry was defined in the other direction
						// if(sideId > 0)
						// 	s_pos = road.attributes.length - s_pos;

						PlannerHNS::WayPoint pRef;
						// iterate trough all geometries defining the road to get the geometry at s_pos
						unsigned int geoIndex = 0;
						for(unsigned int i = 0; i < road.geometry_attributes.size(); i ++)
						{
							if(i < road.geometry_attributes.size()-1)
							{
								if(road.geometry_attributes.at(i)->start_position < s_pos && road.geometry_attributes.at(i + 1)->start_position > s_pos)
									geoIndex = i;
								
							}
							if(i == road.geometry_attributes.size()-1)
							{
								if(road.geometry_attributes.at(i)->start_position < s_pos)
									geoIndex = i;
							}
						}

						const std::unique_ptr<opendrive::GeometryAttributes> &geometry = road.geometry_attributes.at(geoIndex);

						// get the gps position of the traffic Light reference
						try
						{
							switch (geometry->type)
							{
								case opendrive::GeometryType::ARC:
								{
									auto arc = static_cast<opendrive::GeometryAttributesArc *>(geometry.get());
									double sArc = s_pos - arc->start_position;
									if(sArc > arc->length)
										sArc = arc->length;
									pRef = GeneratePointFromArc( arc, &road, *width, laneId, 0, sideId, arc->start_position, sArc);
									break;
								}
								break;
								case opendrive::GeometryType::LINE:
								{
									auto line = static_cast<opendrive::GeometryAttributesLine *>(geometry.get());
									double sLine = s_pos - line->start_position;
									if(sLine > line->length)
										sLine = line->length;
									pRef = GeneratePointFromLine( line, &road, *width, laneId, 0, sideId, line->start_position, sLine );
									break;
								}
								break;
								case opendrive::GeometryType::SPIRAL:
								{
									ROS_FATAL(">> SPIRAL Geometries are not covered by GetCenterLaneData!");
									break;
								}
								break;
								case opendrive::GeometryType::POLY3:
								{
									ROS_FATAL(">> POLY3 Geometries are not covered by GetCenterLaneData!");
									break;
								}
								break;
								case opendrive::GeometryType::PARAMPOLY3:
								{
									ROS_FATAL(">> PARAMPOLY3 Geometries are not covered by GetCenterLaneData!");
									break;
								}
								break;
								default:
								break;
							}
						}
						catch (...)
						{
							ROS_FATAL(">> In GetTrafficLightsList: Geometries are not covered by GetCenterLaneData!");
							continue;
						}

						PlannerHNS::WayPoint p;
						p.pos.x = pRef.pos.x + cos(pRef.pos.a - M_PI_2) * (t_pos); //* sideId);
						p.pos.y = pRef.pos.y + sin(pRef.pos.a - M_PI_2) * (t_pos); //* sideId);
						p.pos.z = road.traffic_signals.at(i).zoffset;

						tl.pose = p;

						tlList.push_back(tl);
					}
				}
			}
		}
	}
	return tsList;
}

std::vector<Curb> OpenDriveMapLoader::GetCurbsList(const opendrive::OpenDriveData* odr){
	std::vector<Curb> cList;
	// first iteration connect everyting based on ids
	for(const opendrive::RoadInformation &road : odr->roads)
    {
		for (int sectionIndex = 0; sectionIndex < road.lanes.lane_sections.size(); sectionIndex++) {
			double endPosition = 0; 
			if(sectionIndex < road.lanes.lane_sections.size()-1)
			{
				endPosition = road.lanes.lane_sections[sectionIndex+1].start_position;
			}
			else
			{
				endPosition = road.attributes.length;
			}
			for(const opendrive::LaneInfo &laneInfoRight : road.lanes.lane_sections[sectionIndex].right)
			{
				int marker_size = laneInfoRight.road_marker.size();
				if(marker_size > 0 && sectionIndex == 0) 
				{
					for ( int j = 0; j < marker_size; j++ ) 
					{
						if(laneInfoRight.road_marker.at(j).type == "curb") 
						{
							OpenDriveMapLoader::openDriveLaneId c_id;
							c_id.roadId = road.attributes.id;
							c_id.laneSectionId = sectionIndex;
							c_id.laneId = laneInfoRight.attributes.id;
							Curb c;
							c.id 		= openDriveIDsToInt(c_id);
							c.laneId 	= openDriveIDsToInt(c_id);
							c.roadId 	= 0;
							c.width		= laneInfoRight.road_marker.at(j).width;
							c.height	= 0.15;
							c.points	= GetLaneData(odr, &road, &laneInfoRight, road.lanes.lane_sections[sectionIndex].start_position, endPosition, 1.0);

							cList.push_back(c);
						}
					}
				}
			}
			for(const opendrive::LaneInfo &laneInfoLeft : road.lanes.lane_sections[sectionIndex].left)
			{
				int marker_size = laneInfoLeft.road_marker.size();
				if(marker_size > 0 && sectionIndex == 0)
				{
					for ( int j = 0; j < marker_size; j++ ) 
					{
						if(laneInfoLeft.road_marker.at(j).type == "curb") 
						{
							OpenDriveMapLoader::openDriveLaneId c_id;
							c_id.roadId = road.attributes.id;
							c_id.laneSectionId = sectionIndex;
							c_id.laneId = laneInfoLeft.attributes.id;
							Curb c;
							c.id 		= openDriveIDsToInt(c_id);
							c.laneId 	= openDriveIDsToInt(c_id);
							c.roadId 	= 0;
							c.width		= laneInfoLeft.road_marker.at(j).width;
							c.height	= 0.15;
							c.points	= GetLaneData(odr, &road, &laneInfoLeft, road.lanes.lane_sections[sectionIndex].start_position, endPosition, 1.0);

							cList.push_back(c);
						}
					}
				}
			}
		}
	}
	return cList;

}


std::vector<opendrive::LaneWidth> OpenDriveMapLoader::GetLaneWidths(	const opendrive::OpenDriveData* odr, 
																		const opendrive::RoadInformation* road, 
																		const opendrive::LaneInfo* laneInfo, 
																		double startPosition, 
																		double factor)
{
	std::vector<opendrive::LaneWidth> lW;
	struct laneIdWidth {
		int id;
		opendrive::LaneWidth laneWidth;
		const inline bool operator==(const laneIdWidth& other) const {
        	return (id==other.id and laneWidth.soffset == other.laneWidth.soffset and laneWidth.a == other.laneWidth.a and laneWidth.b == other.laneWidth.b and laneWidth.c == other.laneWidth.c and laneWidth.d == other.laneWidth.d);
    	}
	};
	std::vector<laneIdWidth> widths;

	for(const opendrive::RoadInformation &r : odr->roads)
    {
		if( r.attributes.id == road->attributes.id)
		{
			for(const opendrive::LaneSection &laneSection : r.lanes.lane_sections)
			{
				if(laneInfo->attributes.id > 0 && laneSection.start_position == startPosition)
				{
					for(const opendrive::LaneInfo &laneInfoLeft : laneSection.left)
					{
						// first add the width of the lane of interest
						if(laneInfoLeft.attributes.id == laneInfo->attributes.id) 
						{
							for(const opendrive::LaneWidth &laneWidth : laneInfoLeft.lane_width)
							{
								laneIdWidth idW;
								idW.laneWidth.soffset = laneWidth.soffset + startPosition;
								if(startPosition > 0 && laneWidth.soffset == 0 && abs(laneInfoLeft.attributes.id) > 1)
								{
									// Getting the lane width of the lane one index closer to the center line: 
									double aOneBelow = 0;
									for(const opendrive::LaneInfo &laneInfoLeftOneBelowLoop : laneSection.right)
									{
										if(abs(laneInfoLeftOneBelowLoop.attributes.id) == (abs(laneInfo->attributes.id) - 1) ) 
										{
											for(const opendrive::LaneWidth &laneWidth : laneInfoLeftOneBelowLoop.lane_width)
											{
												if(laneWidth.soffset == 0)
												{
													aOneBelow = laneWidth.a;
												}
											}
										}
									}
									
									idW.laneWidth.a = ( -1 * aOneBelow) * factor;
									idW.laneWidth.b = laneWidth.b;
									idW.laneWidth.c = laneWidth.c;
									idW.laneWidth.d = laneWidth.d;
								}
								else
								{
									idW.laneWidth.a = laneWidth.a * factor;
									idW.laneWidth.b = laneWidth.b * factor;
									idW.laneWidth.c = laneWidth.c * factor;
									idW.laneWidth.d = laneWidth.d * factor;
								}
								
								idW.id = laneInfoLeft.attributes.id;

								widths.push_back(idW);
							}
							for(const opendrive::LaneInfo &laneInfoLeftLoop : laneSection.left)
							{
								if(abs(laneInfoLeftLoop.attributes.id) < abs(laneInfo->attributes.id)) 
								{
									for(const opendrive::LaneWidth &laneWidth : laneInfoLeftLoop.lane_width)
									{
										laneIdWidth idW;
										idW.laneWidth.soffset = laneWidth.soffset + startPosition;
										idW.laneWidth.a = laneWidth.a;
										idW.laneWidth.b = laneWidth.b;
										idW.laneWidth.c = laneWidth.c;
										idW.laneWidth.d = laneWidth.d;
										idW.id = laneInfoLeftLoop.attributes.id;

										widths.push_back(idW);
									}
								}
							}
							break;
						}
					}
				}

				if(laneInfo->attributes.id < 0 && laneSection.start_position == startPosition)
				{
					for(const opendrive::LaneInfo &laneInfoRight : laneSection.right)
					{
						// first add the width of the lane of interest
						if(laneInfoRight.attributes.id == laneInfo->attributes.id) 
						{
							for(const opendrive::LaneWidth &laneWidth : laneInfoRight.lane_width)
							{
								laneIdWidth idW;
								idW.laneWidth.soffset = laneWidth.soffset + startPosition;
								if(startPosition > 0 && laneWidth.soffset == 0 && abs(laneInfoRight.attributes.id) > 1)
								{
									// Getting the lane width of the lane one index closer to the center line: 
									double aOneBelow = 0;
									for(const opendrive::LaneInfo &laneInfoRightOneBelowLoop : laneSection.right)
									{
										if(abs(laneInfoRightOneBelowLoop.attributes.id) == (abs(laneInfo->attributes.id) - 1) ) 
										{
											for(const opendrive::LaneWidth &laneWidth : laneInfoRightOneBelowLoop.lane_width)
											{
												if(laneWidth.soffset == 0)
												{
													aOneBelow = laneWidth.a;
												}
											}
										}
									}
									
									idW.laneWidth.a = ( -1 * aOneBelow) * factor;
									idW.laneWidth.b = laneWidth.b;
									idW.laneWidth.c = laneWidth.c;
									idW.laneWidth.d = laneWidth.d;
								}
								else
								{
									idW.laneWidth.a = laneWidth.a * factor;
									idW.laneWidth.b = laneWidth.b * factor;
									idW.laneWidth.c = laneWidth.c * factor;
									idW.laneWidth.d = laneWidth.d * factor;
								}
								
								idW.id = laneInfoRight.attributes.id;

								widths.push_back(idW);
							}
							for(const opendrive::LaneInfo &laneInfoRightLoop : laneSection.right)
							{
								if(abs(laneInfoRightLoop.attributes.id) < abs(laneInfo->attributes.id)) 
								{
									for(const opendrive::LaneWidth &laneWidth : laneInfoRightLoop.lane_width)
									{
										laneIdWidth idW;
										idW.laneWidth.soffset = laneWidth.soffset + startPosition;
										idW.laneWidth.a = laneWidth.a;
										idW.laneWidth.b = laneWidth.b;
										idW.laneWidth.c = laneWidth.c;
										idW.laneWidth.d = laneWidth.d;
										idW.id = laneInfoRightLoop.attributes.id;

										widths.push_back(idW);
									}
								}
							}
							break;
						}
					}
				}
			}
		}
	}

	// sort returnWidths
    std::sort(widths.begin(), widths.end(), [](laneIdWidth a, laneIdWidth b) {
		return abs(a.id) < abs(b.id);
    });

	std::vector<double> soffsets;
	for(int i = 0; i < widths.size(); i++)
	{
		soffsets.push_back(widths.at(i).laneWidth.soffset);
	}
	std::vector<double>::iterator ip_s;
	ip_s = std::unique(soffsets.begin(), soffsets.end(), [](double a, double b) {
		return a == b;
    });

	soffsets.resize(std::distance(soffsets.begin(), ip_s));
	std::sort(soffsets.begin(), soffsets.end(), [](double a, double b) {
		return a < b;
    });

	std::vector<laneIdWidth> tempWidths;
	// go trough the unique and sorted soffsets
	for(int i = 0; i < soffsets.size(); i++)
	{
		// go through the laneWidths associated with their lane id
		for(int j = 0; j < widths.size(); j++)
		{
			//find amount of lane width entries with the same id
			std::vector<double> existingSOffsets;
			for(int k = 0; k < widths.size(); k++)
			{
				if(widths.at(k).id == widths.at(j).id)
				{
					existingSOffsets.push_back(widths.at(j).laneWidth.soffset);
				}
			}
			// check if there are as many soffsets as width entries
			if(existingSOffsets.size() == soffsets.size())
			{
				// since there are as many width entries as soffsets append the width entry to the temp laneWidth vector
				tempWidths.push_back(widths.at(j));
			}
			else
			{
				// there must be less width entries
				// therefore new width entries for the non existing soffsets must be created

				// 1: find missing soffsets
				std::vector<double> missingSOffsets;
				for(int l = 0; l < soffsets.size(); l ++)
				{
					bool found = false;
					for(int m = 0; m < existingSOffsets.size(); m ++)
					{
						if(existingSOffsets.at(m) == soffsets.at(l))
						{
							found = true;
							break;
						}
					}
					if(found == false)
					{
						missingSOffsets.push_back(soffsets.at(l));
					}
				}

				// 2: now go trough the missingSOffsets and append new entries
				for(int n = 0; n < missingSOffsets.size(); n ++)
				{
					laneIdWidth width = widths.at(j);
					width.laneWidth.soffset = missingSOffsets.at(n);		
					tempWidths.push_back(width);
				}

				// 3: also go trough existingOffsets
				for(int n = 0; n < existingSOffsets.size(); n ++)
				{
					laneIdWidth width = widths.at(j);
					width.laneWidth.soffset = existingSOffsets.at(n);		
					tempWidths.push_back(width);			
				}
			}
		}	
	}

	// delete duplicate entries in tempWidhts vector
	auto end = tempWidths.end();
	for (auto it = tempWidths.begin(); it != end; ++it) {
		end = std::remove(it + 1, end, *it);
	}
	tempWidths.erase(end, tempWidths.end());

	// Now sum up the width at each soffset and add it to the lW vector
	for(int i = 0; i < soffsets.size(); i++)
	{
		opendrive::LaneWidth w;
		w.soffset = soffsets.at(i);
		w.a = 0;
		w.b = 0;
		w.c = 0;
		w.d = 0;
		for(int j = 0; j < tempWidths.size(); j++)
		{
			if(soffsets.at(i) == tempWidths.at(j).laneWidth.soffset)
			{
				w.a += tempWidths.at(j).laneWidth.a;
				w.b += tempWidths.at(j).laneWidth.b;
				w.c += tempWidths.at(j).laneWidth.c;
				w.d += tempWidths.at(j).laneWidth.d;
			}
		}
		lW.push_back(w);
	}
	
	return lW;
}

std::vector<Boundary> OpenDriveMapLoader::GetBoundariesList(const opendrive::OpenDriveData* odr)
{
	std::vector<Boundary> bList;

	int boundaryIdCounter = 0;
	// loop trough all junctions in odr map
	for(const opendrive::Junction &junction : odr->junctions)
	{
		/*
			Therefore loop through all roads inside the junction.
			Then construct a polygon around the junction.
		*/
		Boundary b;
		b.id = PlannerHNS::INTERSECTION_BOUNDARY;
		boundaryIdCounter ++;
		b.roadId = 0;
		b.type = PlannerHNS::INTERSECTION_BOUNDARY;
		std::vector<WayPoint> points;
		for(const opendrive::JunctionConnection &connection: junction.connections)
		{
			for(const opendrive::RoadInformation &road : odr->roads)
    		{
				if(connection.attributes.connecting_road == road.attributes.id)
				{
					for (int i = 0; i < road.lanes.lane_sections.size(); i++) {
						double endPosition = road.attributes.length;
						if (i + 1 != road.lanes.lane_sections.size())
						{
							endPosition = road.lanes.lane_sections[i+1].start_position;
						}
						for(const opendrive::LaneInfo &laneInfoRight : road.lanes.lane_sections[i].right)
						{
							points = GetLaneData(odr, &road, &laneInfoRight, road.lanes.lane_sections[i].start_position, endPosition, 0.5);
							for(const WayPoint &p : points) 
							{
								b.points.push_back(p);
							}
						}
						for(const opendrive::LaneInfo &laneInfoLeft : road.lanes.lane_sections[i].left)
						{
							points = GetLaneData(odr, &road, &laneInfoLeft, road.lanes.lane_sections[i].start_position, endPosition, 0.5);
							for(const WayPoint &p : points)
							{
								b.points.push_back(p);
							}
						}
					}
				}
			}
		}

		// Gift Wrapping Algorithm for Junction Convex Hull calculation
		std::vector<WayPoint> hullPoints;
		int firstPointOnHullIndex = FindLeftmostPointInJunctionPointsIndex(b.points);
		int pointOnHullIndex = firstPointOnHullIndex;
		WayPoint pointOnHull = b.points.at(pointOnHullIndex);
		int endpointIndex = 0;
		WayPoint endpoint;
		int i = 0;
		do
		{
			hullPoints.push_back(pointOnHull);
			endpoint = b.points.at(0);
			endpointIndex = 0;
			for(int j = 0; j < b.points.size(); j ++)
			{
				if(endpointIndex == pointOnHullIndex || isPointLeftOfLine(hullPoints.at(i), endpoint, b.points.at(j)))
				{
					endpoint = b.points.at(j);
					endpointIndex = j;
				}
			}
			i ++;
			pointOnHull = endpoint;
			pointOnHullIndex = endpointIndex;
		} while (endpointIndex != firstPointOnHullIndex);

		b.points.clear();
		// for clean visualization and closing boundaries, the first point is added as last point for a second time
		hullPoints.push_back(hullPoints.at(0));
		b.points = hullPoints;
		bList.push_back(b);
	}

	return bList;
}

int OpenDriveMapLoader::FindLeftmostPointInJunctionPointsIndex(std::vector<PlannerHNS::WayPoint>& points)
{
	int leftmostPointIndex = 0;
	double leftmostPosY = points[leftmostPointIndex].pos.y;
	for (int i = 0; i < points.size(); i ++)
	{
		if(points[i].pos.y < points[leftmostPointIndex].pos.y)
		{
			leftmostPointIndex = i;
			leftmostPosY = points[i].pos.y;
		}
	}
	return leftmostPointIndex;
}

bool OpenDriveMapLoader::isPointLeftOfLine(PlannerHNS::WayPoint A, PlannerHNS::WayPoint B, PlannerHNS::WayPoint M)
{
	// line is defined by A and B
	// M is the point to check
	bool isLeft = false;

	double determinantABAM = (B.pos.x - A.pos.x) * (M.pos.y - A.pos.y) - (B.pos.y - A.pos.y) * (M.pos.x - A.pos.x);
	if(determinantABAM > 0)
		isLeft = true;
	return isLeft;
}

int OpenDriveMapLoader::openDriveIDsToInt(OpenDriveMapLoader::openDriveLaneId id)
{
	uint64_t laneIDMask = 255;
	return (int)(id.roadId << 16 | id.laneSectionId << 8 | (id.laneId & laneIDMask));
}
OpenDriveMapLoader::openDriveLaneId OpenDriveMapLoader::intToOpenDriveIDs(int id)
{
	openDriveLaneId returnId;
	return returnId;
}

} /* namespace PlannerHNS */