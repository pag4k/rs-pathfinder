extern crate rs_graph;
use rs_graph::Graph;

extern crate rs_binaryheap;
use rs_binaryheap::BinaryHeap;

use std::collections::{HashMap, HashSet};
use std::f32;
use std::hash::Hash;
use std::usize;

use std::cmp::Ordering;

pub trait Walkable {
    fn is_walkable(&self) -> bool;
}

pub trait AStarHeuristic {
    fn get_distance(&Self, &Self) -> f32;
}

pub struct Pathfinder<T> {
    graph: Graph<T, usize>,
    dictionnary: HashMap<T, usize>,
}

impl<T: Clone + Copy + Eq + Hash + Walkable + AStarHeuristic> Pathfinder<T> {
    pub fn from_2dvec(map: &[Vec<T>]) -> Pathfinder<T> {
        let mut graph = Graph::new();
        let mut vertex_map: HashMap<T, usize> = HashMap::new();
        let mut coordinates_map: HashMap<T, (usize, usize)> = HashMap::new();

        map.iter().enumerate().for_each(|(y, row)| {
            row.iter().enumerate().for_each(|(x, element)| {
                if element.is_walkable() {
                    vertex_map.insert(*element, graph.insert_vertex(*element));
                    coordinates_map.insert(*element, (y, x));
                }
            })
        });

        let neighbors: Vec<(isize, isize)> = vec![(1, 0), (0, 1), (-1, 0), (0, -1)];

        vertex_map.iter().for_each(|(vertex, &index)| {
            let (current_y, current_x) = coordinates_map
                .get(vertex)
                .expect("Could not find vertex coordinates.");
            neighbors
                .iter()
                .map(|(dy, dx)| ((*current_y as isize) + dx, (*current_x as isize) + dy))
                .filter(|(y, x)| *y >= 0 && *x >= 0)
                .filter(|(y, x)| {
                    map.get(*y as usize).is_some() && map[*y as usize].get(*x as usize).is_some()
                })
                .map(|(y, x)| &map[y as usize][x as usize])
                .filter(|element| vertex_map.contains_key(element))
                .for_each(|element| {
                    graph.insert_edge(1, index, vertex_map[element]).unwrap();
                })
        });

        Pathfinder {
            graph: graph,
            dictionnary: vertex_map,
        }
    }

    pub fn get_neighbors(&self, element: &T) -> Option<Vec<T>> {
        let vertex_index = match self.dictionnary.get(element) {
            Some(n) => *n,
            None => return None,
        };
        if self.graph.outgoing_edges(vertex_index).is_none()
            || self.graph.outgoing_edges(vertex_index).unwrap().is_empty()
        {
            return None;
        }
        Some(
            self.graph
                .outgoing_edges(vertex_index)
                .unwrap()
                .iter()
                .map(|&edge_index| {
                    self.graph
                        .get_vertex_element(self.graph.opposite(vertex_index, edge_index).unwrap())
                        .unwrap()
                })
                .cloned()
                .collect(),
        )
    }

    //This algorithm is based on https://www3.cs.stonybrook.edu/~rezaul/papers/TR-07-54.pdf.
    //It uses a BinaryHeap that does not have a decrease_key() functions.
    //While this implementation might be less memory efficient since some nodes might end up
    //multiple times in the BinaryHeap, the paper concludes that "using a standard priority queue
    //without the decrease-key operation results in better performance than using one with the
    //decrease-key operation in most cases".
    pub fn dijkstra(&self, start_element: &T, end_element: &T) -> Option<Vec<T>> {
        //Get a reference to the graph to lighten the code.
        let graph = &self.graph;

        //Get indices associated the graph elements.
        let start_index = self.dictionnary.get(start_element);
        let end_index = self.dictionnary.get(end_element);

        //If either cannot be found, abort.
        if start_index.is_none() || end_index.is_none() {
            return None;
        }
        let start_index = start_index.cloned().unwrap();
        let end_index = end_index.cloned().unwrap();

        //If they are are the same, abort.
        if start_index == end_index {
            return None;
        }

        //Instantiate BinaryHeap and add start_index.
        let mut binary_heap: BinaryHeap<VertexDistance> = BinaryHeap::new();
        binary_heap.push(VertexDistance {
            index: start_index,
            distance: 0,
        });

        //Instantiate HashMap to trace back path.
        let mut came_from: HashMap<usize, usize> = HashMap::new();

        //Set all distance to maximum usize value.
        let mut distance: HashMap<usize, usize> = graph
            .vertices()
            .iter()
            .map(|vertex| (vertex.index, usize::MAX))
            .collect();

        //Loop until the BinaryHeap is empty.
        'outer: while !binary_heap.is_empty() {
            let current_vertex = binary_heap.pop().unwrap();
            let current_index: usize = current_vertex.index;
            let current_distance: usize = current_vertex.distance;
            //Check if current element has not already been processed.
            //This check is eseential since some elements can be duplicated, but with different
            //distance. See comment above about implementation.
            if current_distance <= distance.get(&current_index).cloned().unwrap() {
                //Update distance.
                distance.insert(current_index, current_distance);
                //If node has no outgoing edges, skip.
                if graph.outgoing_edges(current_index).is_none() {
                    continue;
                }
                //For each outgoing edge, update came_from and distance if shorter path.
                for edge_index in graph.outgoing_edges(current_index).unwrap().iter().cloned() {
                    //Get opposite node.
                    let opposite_index: usize = graph.opposite(current_index, edge_index).unwrap();
                    //Get potential new distance.
                    let new_distance: usize =
                        distance[&current_index] + graph.get_edge_element(edge_index).unwrap();
                    if new_distance < distance[&opposite_index] {
                        binary_heap.push(VertexDistance {
                            index: opposite_index,
                            distance: new_distance,
                        });
                        distance.insert(opposite_index, new_distance);
                        came_from.insert(opposite_index, current_index);
                        //If done with end node, break.
                        if opposite_index == end_index {
                            break 'outer;
                        }
                    }
                }
            }
        }
        //If no path was found, abort.
        if !came_from.contains_key(&end_index) {
            return None;
        }

        //Build shortest path using came_from.
        let mut path_indices = vec![end_index];
        loop {
            let previous_index: usize = *path_indices.last().unwrap();
            if previous_index == start_index {
                break;
            }
            path_indices.push(came_from[&previous_index]);
        }
        //Reverse, map to node elements, and return.
        Some(
            path_indices
                .iter()
                .rev()
                .map(|&vertex_index| graph.get_vertex_element(vertex_index).unwrap())
                .cloned()
                .collect(),
        )
    }

    pub fn astar_path(&self, start_element: &T, end_element: &T) -> Option<Vec<T>> {
        let delta = 1;

        let mut closed_set: HashSet<T> = HashSet::new();
        let mut open_set: HashSet<T> = HashSet::new();
        open_set.insert(*start_element);
        let mut came_from: HashMap<T, T> = HashMap::new();
        let mut g_score: HashMap<T, f32> = self
            .graph
            .vertices()
            .iter()
            .map(|vertex| (vertex.element, f32::MAX))
            .collect();
        g_score.insert(*start_element, 0 as f32);
        let mut f_score: HashMap<T, f32> = self
            .graph
            .vertices()
            .iter()
            .map(|vertex| (vertex.element, f32::MAX))
            .collect();
        f_score.insert(*start_element, T::get_distance(start_element, end_element));

        loop {
            if open_set.is_empty() {
                break;
            }
            //TODO: Use a priority queue instead. of min.
            let current_element = *open_set
                .iter()
                .min_by(|element1, element2| {
                    f_score[*element1].partial_cmp(&f_score[*element2]).unwrap()
                })
                .unwrap();
            if current_element == *end_element {
                let mut reverse_element = *end_element;
                let mut path = vec![*end_element];
                loop {
                    path.push(came_from[&reverse_element]);
                    reverse_element = came_from[&reverse_element];
                    if reverse_element == *start_element {
                        break;
                    }
                }
                return Some(path.iter().rev().cloned().collect());
            }

            open_set.remove(&current_element);
            closed_set.insert(current_element);

            let current_element_index = self.dictionnary[&current_element];
            if self.graph.outgoing_edges(current_element_index).is_none()
                || self
                    .graph
                    .outgoing_edges(current_element_index)
                    .unwrap()
                    .is_empty()
            {
                continue;
            }
            self.graph
                .outgoing_edges(current_element_index)
                .unwrap()
                .iter()
                .map(|edge_index| self.graph.opposite(current_element_index, *edge_index))
                .filter(|neighbor_index_option| neighbor_index_option.is_some())
                .map(|neighbor_index_option| neighbor_index_option.unwrap())
                .filter(|neighbor_index| self.graph.get_vertex_element(*neighbor_index).is_some())
                .filter(|neighbor_index| {
                    !closed_set.contains(&self.graph.get_vertex_element(*neighbor_index).unwrap())
                })
                .for_each(|neighbor_index| {
                    let neighbor = self.graph.get_vertex_element(neighbor_index).unwrap();
                    let tentative_g_score = g_score[&current_element] + (delta as f32);
                    if !open_set.contains(neighbor) {
                        open_set.insert(*neighbor);
                    }
                    if tentative_g_score < g_score[&neighbor] {
                        came_from.insert(*neighbor, current_element);
                        g_score.insert(*neighbor, tentative_g_score);
                        f_score.insert(
                            *neighbor,
                            g_score[&neighbor] + T::get_distance(neighbor, end_element),
                        );
                    }
                })
        }
        None
    }
}

#[derive(Eq, PartialEq)]
struct VertexDistance {
    index: usize,
    distance: usize,
}

impl Ord for VertexDistance {
    fn cmp(&self, other: &VertexDistance) -> Ordering {
        other
            .distance
            .cmp(&self.distance)
            .then_with(|| self.index.cmp(&other.index))
    }
}

impl PartialOrd for VertexDistance {
    fn partial_cmp(&self, other: &VertexDistance) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

#[cfg(test)]

mod tests {
    use super::*;

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
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
        fn get_distance(point1: &Point2D, point2: &Point2D) -> f32 {
            f32::sqrt(
                (point1.x as f32 - point2.x as f32).powi(2)
                    + (point1.y as f32 - point2.y as f32).powi(2),
            )
        }
    }

    #[test]
    fn test_get_neighbors() {
        let mapvec: Vec<Vec<usize>> = vec![
            vec![0, 0, 0, 0, 1],
            vec![0, 1, 0, 1, 0],
            vec![0, 1, 0, 1, 0],
            vec![0, 1, 1, 1, 0],
            vec![0, 0, 0, 0, 0],
        ];
        let mut map: Vec<Vec<Point2D>> = vec![];
        for y in 0..5 {
            map.push(vec![]);
            for x in 0..5 {
                map[y].push(Point2D {
                    x: x,
                    y: y,
                    walkable: mapvec[y][x] == 1,
                });
            }
        }
        let pathfinder = Pathfinder::from_2dvec(&map);
        assert!(pathfinder.get_neighbors(&map[1][1]).is_some());
        assert!(pathfinder.get_neighbors(&map[2][1]).unwrap().len() == 2);
        assert!(pathfinder
            .get_neighbors(&map[1][3])
            .unwrap()
            .contains(&map[2][3]));
    }

    #[test]
    fn test_a_star() {
        let mapvec: Vec<Vec<usize>> = vec![
            vec![0, 0, 0, 0, 1],
            vec![0, 1, 0, 1, 0],
            vec![0, 1, 0, 1, 0],
            vec![0, 1, 1, 1, 0],
            vec![0, 0, 0, 0, 0],
        ];
        let mut map: Vec<Vec<Point2D>> = vec![];
        for y in 0..5 {
            map.push(vec![]);
            for x in 0..5 {
                map[y].push(Point2D {
                    x: x,
                    y: y,
                    walkable: mapvec[y][x] == 1,
                });
            }
        }
        let pathfinder = Pathfinder::from_2dvec(&map);
        assert!(pathfinder.astar_path(&map[1][1], &map[3][3]).is_some());
        assert!(pathfinder.astar_path(&map[1][1], &map[3][3]).unwrap().len() == 5);
        assert!(pathfinder.astar_path(&map[1][1], &map[3][3]).unwrap()[0] == map[1][1]);
        assert!(pathfinder.astar_path(&map[1][1], &map[3][3]).unwrap()[1] != map[1][1]);
        assert!(
            pathfinder
                .astar_path(&map[1][1], &map[3][3])
                .unwrap()
                .pop()
                .unwrap()
                == map[3][3]
        );
        assert!(
            pathfinder
                .astar_path(&map[1][1], &map[3][3])
                .unwrap()
                .pop()
                .unwrap()
                != map[4][3]
        );
        assert!(pathfinder.astar_path(&map[1][1], &map[4][4]).is_none());
        assert!(pathfinder.astar_path(&map[1][1], &map[0][4]).is_none());
    }

    #[test]
    fn test_dijsktra() {
        let mapvec: Vec<Vec<usize>> = vec![
            vec![0, 0, 0, 0, 1],
            vec![0, 1, 0, 1, 0],
            vec![0, 1, 0, 1, 0],
            vec![0, 1, 1, 1, 0],
            vec![0, 0, 0, 0, 0],
        ];
        let mut map: Vec<Vec<Point2D>> = vec![];
        for y in 0..5 {
            map.push(vec![]);
            for x in 0..5 {
                map[y].push(Point2D {
                    x: x,
                    y: y,
                    walkable: mapvec[y][x] == 1,
                });
            }
        }
        let pathfinder = Pathfinder::from_2dvec(&map);
        assert!(pathfinder.dijkstra(&map[1][1], &map[3][3]).is_some());
        assert!(pathfinder.dijkstra(&map[1][1], &map[3][3]).unwrap().len() == 5);
        assert!(pathfinder.dijkstra(&map[1][1], &map[3][3]).unwrap()[0] == map[1][1]);
        assert!(pathfinder.dijkstra(&map[1][1], &map[3][3]).unwrap()[1] != map[1][1]);
        assert!(
            pathfinder
                .dijkstra(&map[1][1], &map[3][3])
                .unwrap()
                .pop()
                .unwrap()
                == map[3][3]
        );
        assert!(
            pathfinder
                .dijkstra(&map[1][1], &map[3][3])
                .unwrap()
                .pop()
                .unwrap()
                != map[4][3]
        );
        assert!(pathfinder.dijkstra(&map[1][1], &map[4][4]).is_none());
        assert!(pathfinder.dijkstra(&map[1][1], &map[0][4]).is_none());
    }
}
