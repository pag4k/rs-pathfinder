# rs-pathfinder
Simple Rust implementation of some pathfinding algorithms.

## Features
- Generate graph from a 2d vector of generic types implementing Walkable and AStarHeuristic traits.
- Find path between two elements generic type using A* search algorithm.
- Find all elements within some range using a variation of Dijkstra's algorithm.

## Todo
- Add more ways to generate graph.
- Other limitations are due to the used [Graph library](https://github.com/pag4k/rs-graph).
    - Edges have no weight.
    - Graph cannot be changed.

## Authors

* **Pierre-André Gagnon** - *Initial work* - [pag4k](https://github.com/pag4k)

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.