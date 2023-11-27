
  // Configure the argument parser
  args::ArgumentParser parser("CGAL-Polyscope Project");
  args::PositionalList<std::string> inputFilenames(parser, "meshes", "Mesh files in OBJ format.");
  // Parse args
  try {
    parser.ParseCLI(argc, argv);
  } catch (args::Help &h) {
    std::cout << parser;
    return 0;
  } catch (args::ParseError &e) {
    std::cerr << e.what() << std::endl;
    std::cerr << parser;
    return 1;
  }
  // Make sure at least one mesh name was given
  if (inputFilenames.Get().size() == 0) {
    std::cerr << "Please specify at least one mesh file as argument" << std::endl;
    return EXIT_FAILURE;
  }

  // Vector to store mesh and geometry pairs
  std::vector<std::tuple<std::unique_ptr<gcs::SurfaceMesh>, std::unique_ptr<gcs::VertexPositionGeometry>>> meshGeometryPairs;

  // Read and register each mesh file
  for (const std::string& filename : inputFilenames) {
    // Read the mesh file
    auto [mesh, geometry] = readManifoldSurfaceMesh(filename);

    // Register the mesh
    polyscope::registerSurfaceMesh("Mesh " + std::to_string(meshGeometryPairs.size() + 1),
                                   geometry->vertexPositions, mesh->getFaceVertexList());

    // Store the mesh and geometry pair in the vector
    meshGeometryPairs.emplace_back(std::move(mesh), std::move(geometry));
  }