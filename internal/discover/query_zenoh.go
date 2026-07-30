//go:build zenoh

package discover

import (
	"fmt"
	"time"

	zgo "github.com/eclipse-zenoh/zenoh-go/zenoh"

	"conductor.dev/conductor/transport/rmwzenoh"
)

// Query asks the network what is on it. rmw_zenoh advertises every node and
// endpoint as a liveliness token, and a liveliness query returns the ones
// alive now — so this is discovery in one round trip, with no ROS install, no
// CLI to parse, and no daemon whose cache might be stale.
func Query(endpoint string, domain int, timeout time.Duration) (*Graph, error) {
	zgo.InitLoggerFromEnvOr("error")
	cfg := zgo.NewConfigDefault()
	if endpoint != "" {
		if err := cfg.InsertJson5("connect/endpoints", fmt.Sprintf("[%q]", endpoint)); err != nil {
			return nil, fmt.Errorf("zenoh config: %w", err)
		}
	}
	session, err := zgo.Open(cfg, nil)
	if err != nil {
		return nil, fmt.Errorf("zenoh open (is a router running at %s?): %w", endpoint, err)
	}
	defer session.Close(nil)

	// Open returns before the session has reached the router, and a liveliness
	// query asked too early is answered by nobody — which reads as an empty
	// graph rather than as an error, and is exactly the kind of quiet
	// half-answer this command exists to remove. So wait to be connected.
	if err := awaitRouter(session, timeout); err != nil {
		return nil, err
	}

	// Scoped to the domain, because two domains on one router are two graphs
	// and merging them would invent peers that cannot hear each other.
	ke, err := zgo.NewKeyExpr(fmt.Sprintf("%s/%d/**", rmwzenoh.AdminSpace, domain))
	if err != nil {
		return nil, err
	}
	replies, err := session.Liveliness().Get(ke, zgo.NewFifoChannel[zgo.Reply](256),
		&zgo.LivelinessGetOptions{TimeoutMs: uint64(timeout / time.Millisecond)})
	if err != nil {
		return nil, fmt.Errorf("liveliness query: %w", err)
	}

	g := &Graph{Domain: domain, Endpoint: endpoint}
	// The channel closes when the query finalizes; this is only the backstop
	// for a router that accepts the query and never answers it.
	deadline := time.After(timeout + 2*time.Second)
	for {
		select {
		case reply, ok := <-replies:
			if !ok {
				return g, nil
			}
			if reply.Err().IsSome() {
				continue
			}
			sample := reply.Ok().Unwrap()
			token := sample.KeyExpr().String()
			e, err := rmwzenoh.ParseToken(token)
			if err != nil {
				// A token this build cannot read is worth reporting, not
				// worth failing over: the graph is someone else's and may
				// carry entity kinds from a newer rmw_zenoh.
				g.Unreadable = append(g.Unreadable, Unreadable{Token: token, Err: err.Error()})
				continue
			}
			g.Entities = append(g.Entities, e)
		case <-deadline:
			return g, nil
		}
	}
}

// awaitRouter waits for the session to be connected to a router (or, on a
// peer-to-peer graph, to any peer). It reports what it was waiting for when it
// gives up, because "no router at that endpoint" and "a graph with nothing on
// it" are different problems with different fixes.
func awaitRouter(session zgo.Session, timeout time.Duration) error {
	deadline := time.Now().Add(timeout)
	for {
		routers, err := session.RoutersZId()
		if err != nil {
			return fmt.Errorf("zenoh session: %w", err)
		}
		if len(routers) > 0 {
			return nil
		}
		peers, err := session.PeersZId()
		if err != nil {
			return fmt.Errorf("zenoh session: %w", err)
		}
		if len(peers) > 0 {
			return nil
		}
		if time.Now().After(deadline) {
			return fmt.Errorf("no zenoh router or peer answered within %s "+
				"(is rmw_zenohd running, and is the endpoint right?)", timeout)
		}
		time.Sleep(50 * time.Millisecond)
	}
}
