extern crate rs_graph;
use rs_graph::Graph;

use std::f32;
use std::collections::{HashSet, HashMap};
use std::hash::Hash;

pub trait Walkable {
    fn is_walkable(&self) -> bool; 
}

pub trait AStarHeuristic {
    fn get_distance(&Self, &Self) -> f32;
}

pub struct Pathfinder<T> {
    graph: Graph<T>,
    dictionnary: HashMap<T, usize>,
}

impl<T: Clone+Copy+Eq+Hash+Walkable+AStarHeuristic> Pathfinder<T> {
    pub fn from_2dvec(map:&[Vec<T>]) -> Pathfinder<T> {
        let mut graph = Graph::new();
        let mut vertex_map: HashMap<T, usize> = HashMap::new();
        let mut coordinates_map: HashMap<T, (usize,usize)> = HashMap::new();

        map.iter().enumerate().for_each(|(y, row)| {
            row.iter().enumerate().for_each(|(x, element)| {
                if element.is_walkable() {
                    vertex_map.insert(*element, graph.insert_vertex(*element));
                    coordinates_map.insert(*element, (y,x));
                }
            })
        });

        let neighbors: Vec<(isize, isize)> = vec!((1, 0), (0, 1), (-1, 0), (0, -1));

        vertex_map.iter().for_each(|(vertex, &index)| {
            let (current_y,current_x) = coordinates_map.get(vertex).expect("Could not find vertex coordinates.");
            neighbors.iter()
            .map(|(dy,dx)| ((*current_y as isize) + dx, (*current_x as isize) + dy))
            .filter(|(y,x)| *y >= 0 && *x >= 0)
            .filter(|(y,x)| map.get(*y as usize).is_some() && map[*y as usize].get(*x as usize).is_some())
            .map(|(y,x)| &map[y as usize][x as usize])
            .filter(|element| vertex_map.contains_key(element))
            .for_each(|element| { graph.insert_edge(index, vertex_map[element]).unwrap(); })
        });

        Pathfinder { graph: graph, dictionnary: vertex_map, }
    }

    pub fn get_neighbors(&self, element:&T) -> Option<Vec<T>> {
        let vertex_index = match self.dictionnary.get(element) {
            Some(n) => *n,
            None => return None,
        };
        if  self.graph.outgoing_edges(vertex_index).is_none() ||
            self.graph.outgoing_edges(vertex_index).unwrap().is_empty() { return None; }
        Some(self.graph.outgoing_edges(vertex_index).unwrap().iter()
        .map(|&edge_index| self.graph.vertices().get(self.graph.opposite(vertex_index, edge_index).unwrap()).unwrap().element)
        .collect())
    }

    pub fn astar_path(&self, start_element: &T, end_element: &T) -> Option<Vec<T>> {
        let delta = 1;

        let mut closed_set: HashSet<T> = HashSet::new();
        let mut open_set: HashSet<T>  = HashSet::new();
        open_set.insert(*start_element);
        let mut came_from: HashMap<T, T> = HashMap::new();
        let mut g_score: HashMap<T, f32> =  self.graph.vertices().iter().map(|vertex| (vertex.element, f32::MAX)).collect();
        g_score.insert(*start_element, 0 as f32);
        let mut f_score: HashMap<T, f32> =  self.graph.vertices().iter().map(|vertex| (vertex.element, f32::MAX)).collect();
        f_score.insert(*start_element, T::get_distance(start_element, end_element));

        loop {
            if open_set.is_empty() { break; } 

            let current_element = *open_set.iter().min_by(|element1, element2| f_score[*element1].partial_cmp(&f_score[*element2]).unwrap()).unwrap();
            if current_element == *end_element {
                let mut reverse_element = *end_element;
                let mut path = vec!(*end_element);
                loop {
                    path.push(came_from[&reverse_element]);
                    reverse_element = came_from[&reverse_element];
                    if reverse_element == *start_element { break; }
                }
                return Some(path.iter().rev().cloned().collect());
            }

            open_set.remove(&current_element);
            closed_set.insert(current_element);

            let current_element_index = self.dictionnary[&current_element];
            if  self.graph.outgoing_edges(current_element_index).is_none() ||
                self.graph.outgoing_edges(current_element_index).unwrap().is_empty() { continue; }
            self.graph.outgoing_edges(current_element_index).unwrap().iter()
            .map(|edge_index| self.graph.opposite(current_element_index, *edge_index))
            .filter(|neighbor_index_option| neighbor_index_option.is_some()) 
            .map(|neighbor_index_option| neighbor_index_option.unwrap())
            .filter(|neighbor_index| self.graph.get_element(*neighbor_index).is_some())
            .filter(|neighbor_index| !closed_set.contains(&self.graph.get_element(*neighbor_index).unwrap()))
            .for_each(|neighbor_index| {
                let neighbor = self.graph.get_element(neighbor_index).unwrap();
                let tentative_g_score = g_score[&current_element] + (delta as f32);
                if !open_set.contains(neighbor) {
                    open_set.insert(*neighbor);
                }
                if tentative_g_score < g_score[&neighbor] {
                    came_from.insert(*neighbor, current_element);
                    g_score.insert(*neighbor, tentative_g_score);
                    f_score.insert(*neighbor, g_score[&neighbor] + T::get_distance(neighbor, end_element));
                }
            })
        }
        
        None
    }

    pub fn dijkstra_range(&self, element: &T, range: usize) -> Vec<T> {
        if range == 0 || !element.is_walkable() { return vec!(); }
        let delta = 1;

        let mut closed_set: HashSet<T> = HashSet::new();
        let mut open_set: HashSet<T>  = HashSet::new();
        open_set.insert(*element);
        let mut distance: HashMap<T, usize> = HashMap::new();
        distance.insert(*element, 0);
        let mut came_from: HashMap<T, T> = HashMap::new();

        loop {
            if open_set.is_empty() { break; } 

            let current_element = match open_set.iter()
            .find(|element| *distance.get(element).expect("Element is in the open_set Vec, but not in the distance HashMap.") <= range - delta) {
                Some(element) => *element,
                None => {
                    break;
                }
            };

            let current_distance = distance[&current_element];
            open_set.remove(&current_element);
            closed_set.insert(current_element);

            let current_element_index = self.dictionnary[&current_element];
            if  self.graph.outgoing_edges(current_element_index).is_none() ||
                self.graph.outgoing_edges(current_element_index).unwrap().is_empty() { continue; }
            self.graph.outgoing_edges(current_element_index).unwrap().iter()
            .map(|edge_index| self.graph.opposite(current_element_index, *edge_index))
            .filter(|neighbor_index_option| neighbor_index_option.is_some()) 
            .map(|neighbor_index_option| neighbor_index_option.unwrap())
            .filter(|neighbor_index| self.graph.get_element(*neighbor_index).is_some())
            .filter(|neighbor_index| !closed_set.contains(&self.graph.get_element(*neighbor_index).unwrap()))
            .for_each(|neighbor_index| {
                let neighbor = self.graph.get_element(neighbor_index).unwrap();
                if !distance.contains_key(&neighbor) || current_distance+delta < *distance.get(&neighbor).expect("Unreachable code.") {
                    distance.insert(*neighbor, current_distance+delta);
                    came_from.insert(*neighbor, current_element);
                    open_set.insert(*neighbor);
                }
            });
        }

        distance.keys().cloned().collect()
    }
}

#[cfg(test)]

mod tests {
    use super::*;

    #[derive(Clone, Copy, PartialEq, Eq, Hash)]
    struct Point2D {
        x: usize,
        y: usize,
        walkable: bool,
    }

    impl Walkable for Point2D {
        fn is_walkable(&self) -> bool {
            self.walkable
        }
    }

    impl AStarHeuristic for Point2D {
        fn get_distance(point1:&Point2D, point2:&Point2D) -> f32 {
            f32::sqrt((point1.x as f32 - point2.x as f32).powi(2) + (point1.y as f32 - point2.y as f32).powi(2))
        }
    }

    #[test]
    fn test_get_neighbors() {
        let mapvec: Vec<Vec<usize>> = vec!( vec!(0,0,0,0,1),
                                            vec!(0,1,0,1,0),
                                            vec!(0,1,0,1,0),
                                            vec!(0,1,1,1,0),
                                            vec!(0,0,0,0,0));
        let mut map: Vec<Vec<Point2D>> = vec!();
        for y in 0..5 {
            map.push(vec!());
            for x in 0..5 {
                map[y].push(Point2D { x:x, y:y, walkable: mapvec[y][x] == 1, });
            }
        }
        let pathfinder = Pathfinder::from_2dvec(&map);
        assert!(pathfinder.get_neighbors(&map[1][1]).is_some());
        assert!(pathfinder.get_neighbors(&map[2][1]).unwrap().len() == 2);
        assert!(pathfinder.get_neighbors(&map[1][3]).unwrap().contains(&map[2][3]));
    }

    #[test]
    fn test_a_star() {
        let mapvec: Vec<Vec<usize>> = vec!( vec!(0,0,0,0,1),
                                            vec!(0,1,0,1,0),
                                            vec!(0,1,0,1,0),
                                            vec!(0,1,1,1,0),
                                            vec!(0,0,0,0,0));
        let mut map: Vec<Vec<Point2D>> = vec!();
        for y in 0..5 {
            map.push(vec!());
            for x in 0..5 {
                map[y].push(Point2D { x:x, y:y, walkable: mapvec[y][x] == 1, });
            }
        }
        let pathfinder = Pathfinder::from_2dvec(&map);
        assert!(pathfinder.astar_path(&map[1][1], &map[3][3]).is_some());
        assert!(pathfinder.astar_path(&map[1][1], &map[3][3]).unwrap().len() == 5);
        assert!(pathfinder.astar_path(&map[1][1], &map[3][3]).unwrap()[0] == map[1][1]);
        assert!(pathfinder.astar_path(&map[1][1], &map[3][3]).unwrap()[1] != map[1][1]);
        assert!(pathfinder.astar_path(&map[1][1], &map[3][3]).unwrap().pop().unwrap() == map[3][3]);
        assert!(pathfinder.astar_path(&map[1][1], &map[3][3]).unwrap().pop().unwrap() != map[4][3]);
        assert!(pathfinder.astar_path(&map[1][1], &map[4][4]).is_none());
        assert!(pathfinder.astar_path(&map[1][1], &map[0][4]).is_none());
    }

    #[test]
    fn test_dijkstra_range() {
        let mapvec: Vec<Vec<usize>> = vec!( vec!(0,0,0,0,1),
                                            vec!(0,1,0,1,0),
                                            vec!(0,1,0,1,0),
                                            vec!(0,1,1,1,0),
                                            vec!(0,0,0,0,0));
        let mut map: Vec<Vec<Point2D>> = vec!();
        for y in 0..5 {
            map.push(vec!());
            for x in 0..5 {
                map[y].push(Point2D { x:x, y:y, walkable: mapvec[y][x] == 1, });
            }
        }
        let pathfinder = Pathfinder::from_2dvec(&map);
        assert!(pathfinder.dijkstra_range(&map[1][1], 2).len() == 3);
        assert!(pathfinder.dijkstra_range(&map[1][1], 7).len() == 7);
        assert!(pathfinder.dijkstra_range(&map[1][1], 7).contains(&map[1][3]));
        assert!(pathfinder.dijkstra_range(&map[0][4], 10).len() == 1);
        assert!(pathfinder.dijkstra_range(&map[1][1], 0).is_empty());
        assert!(pathfinder.dijkstra_range(&map[0][0], 10).is_empty());
    }
}
